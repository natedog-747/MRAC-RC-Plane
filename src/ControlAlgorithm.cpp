#include "ControlAlgorithm.h"
#include <Arduino.h>  // for constrain
#include <cmath>

float ControlAlgorithm::computeCommand(ControlData& state) {
  // LQR-style state feedback: u = -K * (x - x_ref)
  static const Matrix<1, 3> K = {0.75f, 0.1f, 0.1f}; // tune as needed
  static float yawRefDeg = 0.0f;                     // reference yaw angle (deg)
  static constexpr float yawRateRefDeg = 0.0f;       // reference yaw rate
  static float integralYawError = 0.0f;              // integral of yaw error
  static float servoCenter = 85.0f;
  static float maxServoDeflection = 25.0f;            // max servo deflection
  static float maxIntegralError = 10.0f;               // max integral error (deg)
  const float dt = state.dtSec;

  auto wrapDeg = [](float deg) -> float {
    // Wrap to (-180, 180]
    while (deg > 180.0f) deg -= 360.0f;
    while (deg <= -180.0f) deg += 360.0f;
    return deg;
  };

  const float yaw = state.eulerYawDeg;
  const float yawRate = state.eulerYawRateDegPerSec;

  const float yawError = wrapDeg(yaw - yawRefDeg);
  if (dt > 0.0f) {
    integralYawError += yawError * dt;
  }
  state.yawErrorIntegralDegSec = integralYawError;

  // Error state vector directly (avoids separate x/xRef construction)
  Matrix<3, 1> xErr = {yawError, yawRate - yawRateRefDeg, integralYawError};
  Matrix<1, 1> uMat = -K * xErr;
  float u = uMat(0, 0);

  // Offset to servo range and clamp.
  u += servoCenter;
  u = constrain(u, servoCenter - maxServoDeflection, servoCenter + maxServoDeflection);
  return u;
}
