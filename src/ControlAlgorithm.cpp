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

void ControlAlgorithm::configureStepCommand(int steps, float stepDeg, float stepDurationSec) {
  totalSteps_ = (steps < 0) ? 0 : steps;
  stepSizeDeg_ = stepDeg;
  stepDurationSec_ = (stepDurationSec > 0.0f) ? stepDurationSec : 1.0f;
  completedSteps_ = 0;
  stepElapsedSec_ = 0.0f;
  stepHighPhase_ = false;
}

float ControlAlgorithm::yawReferenceDeg() const {
  return yawRefDeg_;
}

void ControlAlgorithm::setYawReference(float yawDeg) {
  yawRefDeg_ = wrapTo360(yawDeg);
  baseYawRefDeg_ = yawRefDeg_;
  integralYawError_ = 0.0f;
  completedSteps_ = 0;
  stepElapsedSec_ = 0.0f;
  stepHighPhase_ = false;
}

float ControlAlgorithm::computeCommand(ControlData& state) {
  const float dt = state.dtSec;

  const float yaw = state.eulerYawDeg;
  const float yawRate = state.eulerYawRateDegPerSec;

  // Advance the reference by step increments if configured.
  if (dt > 0.0f && totalSteps_ > 0 && stepDurationSec_ > 0.0f) {
    stepElapsedSec_ += dt;
    while (completedSteps_ < totalSteps_ && stepElapsedSec_ >= stepDurationSec_) {
      stepElapsedSec_ -= stepDurationSec_;
      ++completedSteps_;
      stepHighPhase_ = !stepHighPhase_;
      yawRefDeg_ = wrapTo360(baseYawRefDeg_ + (stepHighPhase_ ? stepSizeDeg_ : 0.0f));
    }
    // After all steps are done, return to base heading.
    if (completedSteps_ >= totalSteps_) {
      yawRefDeg_ = baseYawRefDeg_;
    }
  }

  // Normalize yaw to [0, 360) to support commands that cross 180/-180 boundaries.
  const float yaw360 = wrapTo360(yaw);
  const float yawError = wrapErrorDeg(yaw360, yawRefDeg_);
  if (dt > 0.0f) {
    integralYawError_ += yawError * dt;
  }
  state.yawRefDeg = yawRefDeg_;
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
