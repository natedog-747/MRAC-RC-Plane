#pragma once

#include <Arduino.h>
#include "TaskBase.h"
#include "ControlData.h"
#include "freertos/queue.h"

// Skeleton logger pinned to a core; replace Serial printing with SD writes.
class SdLoggerTask : public TaskBase {
 public:
  SdLoggerTask(QueueHandle_t dataQueue, BaseType_t core = 1);
  void setQueue(QueueHandle_t dataQueue);

 protected:
  void run() override;

 private:
  QueueHandle_t dataQueue_;
};
