#include "ControlServoTask.h"

ControlServoTask::ControlServoTask(int inPin,
                                   int outPin,
                                   int overridePin,
                                   uint32_t overrideThresholdUs,
                                   int overrideAngleDeg,
                                   uint16_t servoMinUs,
                                   uint16_t servoMaxUs,
                                   QueueHandle_t dataQueue)
    : TaskBase("Control_Servo", 4096, 1, 0),
      inPin_(inPin),
      outPin_(outPin),
      overridePin_(overridePin),
      overrideThresholdUs_(overrideThresholdUs),
      overrideAngleDeg_(overrideAngleDeg),
      servoMinUs_(servoMinUs),
      servoMaxUs_(servoMaxUs),
      dataQueue_(dataQueue) {}

volatile bool& ControlServoTask::overrideFlag() {
  return overrideActive_;
}

void ControlServoTask::setDataQueue(QueueHandle_t queue) {
  dataQueue_ = queue;
}

void ControlServoTask::run() {
  pinMode(inPin_, INPUT);
  pinMode(overridePin_, INPUT);

  // Attach servo output; min/max constrain the microsecond range.
  servo_.attach(outPin_, servoMinUs_, servoMaxUs_);

  for (;;) {
    // Measure the override channel first; use a small state machine to require consecutive samples.
    uint32_t overrideUs = pulseIn(overridePin_, HIGH, kPulseTimeoutUs);
    bool overrideMeasurement = (overrideUs > kMinPulseUs) && (overrideUs >= overrideThresholdUs_);

    switch (overrideState_) {
      case OverrideState::Mirroring:
        if (overrideMeasurement) {
          overrideTrueCount_ = 1;
          overrideState_ = (kOverrideConfirmations == 1) ? OverrideState::OverrideActive
                                                         : OverrideState::ConfirmingOverride;
        } else {
          overrideTrueCount_ = overrideFalseCount_ = 0;
          overrideActive_ = false;
        }
        break;

      case OverrideState::ConfirmingOverride:
        if (overrideMeasurement) {
          if (++overrideTrueCount_ >= kOverrideConfirmations) {
            overrideState_ = OverrideState::OverrideActive;
            overrideActive_ = true;
            overrideTrueCount_ = overrideFalseCount_ = 0;
          }
        } else {
          overrideState_ = OverrideState::Mirroring;
          overrideTrueCount_ = overrideFalseCount_ = 0;
          overrideActive_ = false;
        }
        break;

      case OverrideState::OverrideActive:
        if (overrideMeasurement) {
          overrideFalseCount_ = 0;
        } else {
          overrideFalseCount_ = 1;
          overrideState_ = (kOverrideConfirmations == 1) ? OverrideState::Mirroring
                                                         : OverrideState::ConfirmingMirror;
        }
        break;

      case OverrideState::ConfirmingMirror:
        if (!overrideMeasurement) {
          if (++overrideFalseCount_ >= kOverrideConfirmations) {
            overrideState_ = OverrideState::Mirroring;
            overrideActive_ = false;
            overrideTrueCount_ = overrideFalseCount_ = 0;
          }
        } else {
          overrideState_ = OverrideState::OverrideActive;
          overrideActive_ = true;
          overrideFalseCount_ = 0;
        }
        break;
    }

    int targetAngle = -1;
    uint32_t targetUs = 0;

    if (overrideActive_) {
      // Run placeholder estimation + control; send to logger.
      uint32_t nowMs = millis();
      ControlData state = estimator_.estimate(nowMs);
      state.overrideActive = true;
      state.controlSignal = controller_.computeCommand(state);

      // Publish to logger if queue available.
      if (dataQueue_) {
        xQueueSend(dataQueue_, &state, 0);
      }

      // While control code is a skeleton, hold servo at neutral (90 deg).
      targetAngle = constrain(static_cast<int>(state.controlSignal), 0, 180);
    } else {
      // Measure the high time of the incoming RC pulse (blocking up to timeout).
      uint32_t highUs = pulseIn(inPin_, HIGH, kPulseTimeoutUs);
      if (highUs > kMinPulseUs) {
        targetUs = constrain(highUs, static_cast<uint32_t>(servoMinUs_),
                             static_cast<uint32_t>(servoMaxUs_));
      } else {
        // No valid pulse: keep last duty and avoid tight spinning.
        vTaskDelay(pdMS_TO_TICKS(5));
      }
    }

    if (targetAngle >= 0) {
      servo_.write(targetAngle);
    } else if (targetUs > 0) {
      // Mirror by writing the measured pulse width directly.
      servo_.writeMicroseconds(targetUs);
    }

    // Yield briefly to let the idle task run so the watchdog is satisfied.
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
