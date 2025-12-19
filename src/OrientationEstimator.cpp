#include <Arduino.h>
#include "OrientationEstimator.h"

namespace {
float degFromQuatWxyz(const float q[4], int axis) {
  const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  switch (axis) {
    case 0:  // roll
      return RAD_TO_DEG * atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    case 1:  // pitch
      return RAD_TO_DEG * asinf(2.0f * (q0 * q2 - q3 * q1));
    case 2:  // yaw
      return RAD_TO_DEG * atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    default:
      return 0.0f;
  }
}

void normalizeQuat(float q[4]) {
  const float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (norm > 0.0f) {
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
  }
}

void initializeQuatFromAccel(Adafruit_BNO055& bno, float q[4]) {
  const int samples = 100;
  float xSum = 0.0f, ySum = 0.0f, zSum = 0.0f;
  sensors_event_t accelEvent;
  for (int i = 0; i < samples; ++i) {
    bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // Coordinate transform similar to provided example: x, -y, z.
    xSum += -accelEvent.acceleration.x;
    ySum += accelEvent.acceleration.y;
    zSum += accelEvent.acceleration.z;
    delay(10);
  }
  const float xAvg = xSum / samples;
  const float yAvg = ySum / samples;
  const float zAvg = zSum / samples;

  const float pitch = -atanf(xAvg / zAvg);
  const float yaw = 0.0f;
  const float roll = atanf(yAvg /zAvg);

  const float cx = cosf(roll * 0.5f);
  const float sx = sinf(roll * 0.5f);
  const float cy = cosf(pitch * 0.5f);
  const float sy = sinf(pitch * 0.5f);
  const float cz = cosf(yaw * 0.5f);
  const float sz = sinf(yaw * 0.5f);

  q[0] = cx * cy * cz + sx * sy * sz;
  q[1] = sx * cy * cz - cx * sy * sz;
  q[2] = cx * sy * cz + sx * cy * sz;
  q[3] = cx * cy * sz - sx * sy * cz;
  normalizeQuat(q);
}
}  // namespace

bool OrientationEstimator::begin() {
  if (ready_) {
    return true;
  }

  if (!bno_.begin()) {
    if (!warned_) {
      Serial.println("BNO055 not detected; check wiring.");
      warned_ = true;
    }
    return false;
  }

  bno_.setExtCrystalUse(true);
  initializeQuatFromAccel(bno_, quat_);
  lastUpdateMs_ = millis();
  ready_ = true;
  warned_ = false;
  Serial.println("BNO055 ready.");
  return true;
}

ControlData OrientationEstimator::estimate(uint32_t nowMs) {
  ControlData data;
  data.timestampMs = nowMs;

  if (!ready_) {
    // Try to initialize once more in case the sensor came up late.
    if (!begin()) {
      return data;
    }
  }

  float dtSec = 0.0f;
  if (lastUpdateMs_ == 0) {
    lastUpdateMs_ = nowMs;
  } else {
    dtSec = (nowMs - lastUpdateMs_) / 1000.0f;
    lastUpdateMs_ = nowMs;
  }

  sensors_event_t gyroEvent;
  sensors_event_t accelEvent;

  bno_.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno_.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Coordinate transforms similar to provided example: roll=-x, pitch=y, yaw=-z for gyro; accel x,-y,z.
  const float w1 = gyroEvent.gyro.x;
  const float w2 = -gyroEvent.gyro.y;
  const float w3 = -gyroEvent.gyro.z;

  lastGyroRadPerSec_.x() = w1;
  lastGyroRadPerSec_.y() = w2;
  lastGyroRadPerSec_.z() = w3;

  lastAccelMps2_.x() = -accelEvent.acceleration.x;
  lastAccelMps2_.y() = accelEvent.acceleration.y;
  lastAccelMps2_.z() = accelEvent.acceleration.z;

  if (dtSec > 0.0f) {
    const float q0 = quat_[0], q1 = quat_[1], q2 = quat_[2], q3 = quat_[3];
    const float dq0 = 0.5f * (-w1 * q1 - w2 * q2 - w3 * q3);
    const float dq1 = 0.5f * ( w1 * q0 + w3 * q2 - w2 * q3);
    const float dq2 = 0.5f * ( w2 * q0 - w3 * q1 + w1 * q3);
    const float dq3 = 0.5f * ( w3 * q0 + w2 * q1 - w1 * q2);

    quat_[0] = q0 + dq0 * dtSec;
    quat_[1] = q1 + dq1 * dtSec;
    quat_[2] = q2 + dq2 * dtSec;
    quat_[3] = q3 + dq3 * dtSec;
    normalizeQuat(quat_);
  }

  lastEulerDeg_.x() = degFromQuatWxyz(quat_, 0);
  lastEulerDeg_.y() = degFromQuatWxyz(quat_, 1);
  lastEulerDeg_.z() = degFromQuatWxyz(quat_, 2);

  data.accelXMps2 = lastAccelMps2_.x();
  data.accelYMps2 = lastAccelMps2_.y();
  data.accelZMps2 = lastAccelMps2_.z();
  data.gyroXRadPerSec = lastGyroRadPerSec_.x();
  data.gyroYRadPerSec = lastGyroRadPerSec_.y();
  data.gyroZRadPerSec = lastGyroRadPerSec_.z();
  data.eulerRollDeg = lastEulerDeg_.x();
  data.eulerPitchDeg = lastEulerDeg_.y();
  data.eulerYawDeg = lastEulerDeg_.z();
  return data;
}
#include <Arduino.h>
#include "OrientationEstimator.h"

namespace {
float degFromQuatWxyz(const float q[4], int axis) {
  const float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];
  switch (axis) {
    case 0:  // roll
      return RAD_TO_DEG * atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
    case 1:  // pitch
      return RAD_TO_DEG * asinf(2.0f * (q0 * q2 - q3 * q1));
    case 2:  // yaw
      return RAD_TO_DEG * atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));
    default:
      return 0.0f;
  }
}

void normalizeQuat(float q[4]) {
  const float norm = sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
  if (norm > 0.0f) {
    q[0] /= norm;
    q[1] /= norm;
    q[2] /= norm;
    q[3] /= norm;
  }
}

void initializeQuatFromAccel(Adafruit_BNO055& bno, float q[4]) {
  const int samples = 100;
  float xSum = 0.0f, ySum = 0.0f, zSum = 0.0f;
  sensors_event_t accelEvent;
  for (int i = 0; i < samples; ++i) {
    bno.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    // Coordinate transform similar to provided example: x, -y, z.
    xSum += -accelEvent.acceleration.x;
    ySum += accelEvent.acceleration.y;
    zSum += accelEvent.acceleration.z;
    delay(10);
  }
  const float xAvg = xSum / samples;
  const float yAvg = ySum / samples;
  const float zAvg = zSum / samples;

  const float pitch = -atanf(xAvg / zAvg);
  const float yaw = 0.0f;
  const float roll = atanf(yAvg / zAvg);

  const float cx = cosf(roll * 0.5f);
  const float sx = sinf(roll * 0.5f);
  const float cy = cosf(pitch * 0.5f);
  const float sy = sinf(pitch * 0.5f);
  const float cz = cosf(yaw * 0.5f);
  const float sz = sinf(yaw * 0.5f);

  q[0] = cx * cy * cz + sx * sy * sz;
  q[1] = sx * cy * cz - cx * sy * sz;
  q[2] = cx * sy * cz + sx * cy * sz;
  q[3] = cx * cy * sz - sx * sy * cz;
  normalizeQuat(q);
}

float wrapDiffDeg(float newDeg, float oldDeg) {
  float diff = newDeg - oldDeg;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}
}  // namespace

bool OrientationEstimator::begin() {
  if (ready_) {
    return true;
  }

  if (!bno_.begin()) {
    if (!warned_) {
      Serial.println("BNO055 not detected; check wiring.");
      warned_ = true;
    }
    return false;
  }

  bno_.setExtCrystalUse(true);
  initializeQuatFromAccel(bno_, quat_);
  lastUpdateMs_ = millis();
  ready_ = true;
  warned_ = false;
  Serial.println("BNO055 ready.");
  return true;
}

ControlData OrientationEstimator::estimate(uint32_t nowMs) {
  ControlData data;
  data.timestampMs = nowMs;

  if (!ready_) {
    // Try to initialize once more in case the sensor came up late.
    if (!begin()) {
      return data;
    }
  }

  float dtSec = 0.0f;
  if (lastUpdateMs_ == 0) {
    lastUpdateMs_ = nowMs;
  } else {
    dtSec = (nowMs - lastUpdateMs_) / 1000.0f;
    lastUpdateMs_ = nowMs;
  }

  sensors_event_t gyroEvent;
  sensors_event_t accelEvent;

  bno_.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);
  bno_.getEvent(&accelEvent, Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Coordinate transforms: roll=-x, pitch=y, yaw=-z for gyro; accel -x,y,z.
  const float w1 = gyroEvent.gyro.x;
  const float w2 = -gyroEvent.gyro.y;
  const float w3 = -gyroEvent.gyro.z;

  lastGyroRadPerSec_.x() = w1;
  lastGyroRadPerSec_.y() = w2;
  lastGyroRadPerSec_.z() = w3;

  lastAccelMps2_.x() = -accelEvent.acceleration.x;
  lastAccelMps2_.y() = accelEvent.acceleration.y;
  lastAccelMps2_.z() = accelEvent.acceleration.z;

  if (dtSec > 0.0f) {
    const float q0 = quat_[0], q1 = quat_[1], q2 = quat_[2], q3 = quat_[3];
    const float dq0 = 0.5f * (-w1 * q1 - w2 * q2 - w3 * q3);
    const float dq1 = 0.5f * ( w1 * q0 + w3 * q2 - w2 * q3);
    const float dq2 = 0.5f * ( w2 * q0 - w3 * q1 + w1 * q3);
    const float dq3 = 0.5f * ( w3 * q0 + w2 * q1 - w1 * q2);

    quat_[0] = q0 + dq0 * dtSec;
    quat_[1] = q1 + dq1 * dtSec;
    quat_[2] = q2 + dq2 * dtSec;
    quat_[3] = q3 + dq3 * dtSec;
    normalizeQuat(quat_);
  }

  lastEulerDeg_.x() = degFromQuatWxyz(quat_, 0);
  lastEulerDeg_.y() = degFromQuatWxyz(quat_, 1);
  lastEulerDeg_.z() = degFromQuatWxyz(quat_, 2);
  if (dtSec > 0.0f) {
    lastEulerRateDegPerSec_.x() = wrapDiffDeg(lastEulerDeg_.x(), prevEulerDeg_.x()) / dtSec;
    lastEulerRateDegPerSec_.y() = wrapDiffDeg(lastEulerDeg_.y(), prevEulerDeg_.y()) / dtSec;
    lastEulerRateDegPerSec_.z() = wrapDiffDeg(lastEulerDeg_.z(), prevEulerDeg_.z()) / dtSec;
  } else {
    lastEulerRateDegPerSec_.x() = 0.0f;
    lastEulerRateDegPerSec_.y() = 0.0f;
    lastEulerRateDegPerSec_.z() = 0.0f;
  }
  prevEulerDeg_ = lastEulerDeg_;

  data.accelXMps2 = lastAccelMps2_.x();
  data.accelYMps2 = lastAccelMps2_.y();
  data.accelZMps2 = lastAccelMps2_.z();
  data.gyroXRadPerSec = lastGyroRadPerSec_.x();
  data.gyroYRadPerSec = lastGyroRadPerSec_.y();
  data.gyroZRadPerSec = lastGyroRadPerSec_.z();
  data.eulerRollDeg = lastEulerDeg_.x();
  data.eulerPitchDeg = lastEulerDeg_.y();
  data.eulerYawDeg = lastEulerDeg_.z();
  data.eulerRollRateDegPerSec = lastEulerRateDegPerSec_.x();
  data.eulerPitchRateDegPerSec = lastEulerRateDegPerSec_.y();
  data.eulerYawRateDegPerSec = lastEulerRateDegPerSec_.z();
  data.dtSec = dtSec;
  return data;
}
