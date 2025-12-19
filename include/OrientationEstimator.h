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

  float gyroX() const { return lastGyroRadPerSec_.x(); }
  float gyroY() const { return lastGyroRadPerSec_.y(); }
  float gyroZ() const { return lastGyroRadPerSec_.z(); }
  float accelX() const { return lastAccelMps2_.x(); }
  float accelY() const { return lastAccelMps2_.y(); }
  float accelZ() const { return lastAccelMps2_.z(); }
  float eulerRoll() const { return lastEulerDeg_.x(); }
  float eulerPitch() const { return lastEulerDeg_.y(); }
  float eulerYaw() const { return lastEulerDeg_.z(); }
  float eulerRollRate() const { return lastEulerRateDegPerSec_.x(); }
  float eulerPitchRate() const { return lastEulerRateDegPerSec_.y(); }
  float eulerYawRate() const { return lastEulerRateDegPerSec_.z(); }

 private:
  Adafruit_BNO055 bno_{55};
  float quat_[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w, x, y, z
  uint32_t lastUpdateMs_ = 0;
  imu::Vector<3> lastAccelMps2_{0, 0, 0};
  imu::Vector<3> lastGyroRadPerSec_{0, 0, 0};
  imu::Vector<3> lastEulerDeg_{0, 0, 0};
  imu::Vector<3> prevEulerDeg_{0, 0, 0};
  imu::Vector<3> lastEulerRateDegPerSec_{0, 0, 0};
  bool ready_ = false;
  bool warned_ = false;
};
