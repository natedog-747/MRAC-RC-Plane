#pragma once

#include <Arduino.h>

// Shared data structure passed between cores when override is active.
struct ControlData {
  uint32_t timestampMs = 0;
  float accelXMps2 = 0.0f;
  float accelYMps2 = 0.0f;
  float accelZMps2 = 0.0f;
  float gyroXRadPerSec = 0.0f;
  float gyroYRadPerSec = 0.0f;
  float gyroZRadPerSec = 0.0f;
  float eulerRollDeg = 0.0f;
  float eulerPitchDeg = 0.0f;
  float eulerYawDeg = 0.0f;
  float eulerRollRateDegPerSec = 0.0f;
  float eulerPitchRateDegPerSec = 0.0f;
  float eulerYawRateDegPerSec = 0.0f;
  float yawErrorIntegralDegSec = 0.0f;
  float dtSec = 0.0f;
  float controlSignal = 0.0f;   // e.g., actuator command or servo angle
  bool overrideActive = false;
};
