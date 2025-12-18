#include "CounterTask.h"

CounterTask::CounterTask(volatile bool& overrideFlag)
    : TaskBase("Core1Count", 2048, 1, 1), overrideFlag_(&overrideFlag) {}

void CounterTask::run() {
  int counter = 1;
  for (;;) {
    if (overrideFlag_ && *overrideFlag_) {
      Serial.printf("Count: %d\n", counter++);
      vTaskDelay(pdMS_TO_TICKS(1000));
    } else {
      // Idle lightly while waiting for override to engage.
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}
