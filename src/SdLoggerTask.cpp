#include "SdLoggerTask.h"

SdLoggerTask::SdLoggerTask(QueueHandle_t dataQueue, BaseType_t core)
    : TaskBase("SdLogger", 4096, 1, core), dataQueue_(dataQueue) {}

void SdLoggerTask::setQueue(QueueHandle_t dataQueue) {
  dataQueue_ = dataQueue;
}

void SdLoggerTask::run() {
  ControlData incoming;
  for (;;) {
    if (dataQueue_ && xQueueReceive(dataQueue_, &incoming, pdMS_TO_TICKS(100)) == pdPASS) {
      // TODO: replace with SD logging; currently print to Serial for visibility.
      Serial.printf("[LOG] t=%lu roll=%.2f pitch=%.2f yaw=%.2f cmd=%.2f override=%d\n",
                    static_cast<unsigned long>(incoming.timestampMs),
                    incoming.rollDeg,
                    incoming.pitchDeg,
                    incoming.yawDeg,
                    incoming.controlSignal,
                    incoming.overrideActive ? 1 : 0);
    }
  }
}
