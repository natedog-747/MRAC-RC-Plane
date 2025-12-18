#pragma once

#include <Arduino.h>
#include <Servo.h>
#include "TaskBase.h"

// Task that reads RC pulses and drives a servo with override handling.
class ControlServoTask : public TaskBase {
 public:
  ControlServoTask(int inPin,
                   int outPin,
                   int overridePin,
                   uint32_t overrideThresholdUs = 1750,
                   int overrideAngleDeg = 90,
                   uint16_t servoMinUs = 1000,
                   uint16_t servoMaxUs = 2000);

  // Exposes a flag others can watch to know when override is active.
  volatile bool& overrideFlag();

 protected:
  void run() override;

 private:
  static constexpr uint32_t kPulseTimeoutUs = 30000;     // allows for >20 ms if signal hiccups
  static constexpr uint32_t kMinPulseUs = 500;           // ignore noise below this
  static constexpr uint8_t kOverrideConfirmations = 3;   // consecutive readings required to toggle override

  enum class OverrideState { Mirroring, ConfirmingOverride, OverrideActive, ConfirmingMirror };

  int inPin_;
  int outPin_;
  int overridePin_;
  uint32_t overrideThresholdUs_;
  int overrideAngleDeg_;
  uint16_t servoMinUs_;
  uint16_t servoMaxUs_;

  Servo servo_;
  volatile bool overrideActive_ = false;
  OverrideState overrideState_ = OverrideState::Mirroring;
  uint8_t overrideTrueCount_ = 0;
  uint8_t overrideFalseCount_ = 0;
};
