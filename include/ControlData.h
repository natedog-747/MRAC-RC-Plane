#pragma once

#include <Arduino.h>

// Shared data structure passed between cores when override is active.
struct ControlData {
  uint32_t timestampMs = 0;
  float rollDeg = 0.0f;
  float pitchDeg = 0.0f;
  float yawDeg = 0.0f;
  float controlSignal = 0.0f;   // e.g., actuator command or servo angle
  bool overrideActive = false;
};
