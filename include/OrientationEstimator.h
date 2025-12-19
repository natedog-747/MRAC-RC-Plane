#pragma once

#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "ControlData.h"

// Wraps the BNO055 IMU to provide roll/pitch/yaw estimates.
class OrientationEstimator {
 public:
  OrientationEstimator() = default;

  // Initialize the IMU; safe to call repeatedly.
  bool begin();

  // Update orientation estimate; returns zeros if the IMU is unavailable.
  ControlData estimate(uint32_t nowMs);

 private:
  Adafruit_BNO055 bno_{55};
  bool ready_ = false;
  bool warned_ = false;
};
