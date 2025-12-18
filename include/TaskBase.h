#pragma once

#include <Arduino.h>
#include "freertos/FreeRTOS.h"

// Lightweight wrapper to create and pin a FreeRTOS task to a core.
class TaskBase {
 public:
  TaskBase(const char* name, uint32_t stackSize, UBaseType_t priority, BaseType_t core)
      : name_(name), stackSize_(stackSize), priority_(priority), core_(core) {}

  bool start() {
    return xTaskCreatePinnedToCore(&TaskBase::taskEntry, name_, stackSize_, this, priority_, &handle_, core_) == pdPASS;
  }

 protected:
  virtual void run() = 0;  // implemented by subclasses

 private:
  static void taskEntry(void* param) { static_cast<TaskBase*>(param)->run(); }

  TaskHandle_t handle_ = nullptr;
  const char* name_;
  uint32_t stackSize_;
  UBaseType_t priority_;
  BaseType_t core_;
};
