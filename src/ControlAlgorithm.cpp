#include "ControlAlgorithm.h"
#include <Arduino.h>  // for constrain
#include <cmath>

namespace {
const Matrix<1, 3> kGain = {1.0f, 0.20f, 0.13f};  // tune as needed
constexpr float kServoCenter = 85.0f;
constexpr float kMaxServoDeflection = 25.0f;
}  // namespace

float ControlAlgorithm::wrapTo360(float deg) {
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  return deg;
}

float ControlAlgorithm::wrapErrorDeg(float actual360, float ref360) {
  float diff = actual360 - ref360;
  while (diff > 180.0f) diff -= 360.0f;
  while (diff < -180.0f) diff += 360.0f;
  return diff;
}

void ControlAlgorithm::setYawReference(float yawDeg) {
  yawRefDeg_ = wrapTo360(yawDeg);
  integralYawError_ = 0.0f;
}

float ControlAlgorithm::computeCommand(ControlData& state) {
  const float dt = state.dtSec;

  const float yaw = state.eulerYawDeg;
  const float yawRate = state.eulerYawRateDegPerSec;

  // Normalize yaw to [0, 360) to support commands that cross 180/-180 boundaries.
  const float yaw360 = wrapTo360(yaw);
  const float yawError = wrapErrorDeg(yaw360, yawRefDeg_);
  if (dt > 0.0f) {
    integralYawError_ += yawError * dt;
  }
  state.yawErrorIntegralDegSec = integralYawError_;

  // Error state vector directly (avoids separate x/xRef construction)
  Matrix<3, 1> xErr = {yawError, yawRate - yawRateRefDeg_, integralYawError_};
  Matrix<1, 1> uMat = -kGain * xErr;
  float u = uMat(0, 0);

  // Offset to servo range and clamp.
  u += kServoCenter;
  u = constrain(u, kServoCenter - kMaxServoDeflection, kServoCenter + kMaxServoDeflection);
  return u;
}
