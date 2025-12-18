#pragma once

#include <Arduino.h>
#include "TaskBase.h"

// Simple counter/logging task pinned to a chosen core.
class CounterTask : public TaskBase {
 public:
  explicit CounterTask(volatile bool& overrideFlag);

 protected:
  void run() override;

 private:
  volatile bool* overrideFlag_ = nullptr;
};
