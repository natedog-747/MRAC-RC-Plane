#pragma once

#include <BasicLinearAlgebra.h>
using namespace BLA;
#include "ControlData.h"

// Skeleton control algorithm; fill in with your control logic.
class ControlAlgorithm {
 public:
  ControlAlgorithm() = default;

  // Compute actuator command based on estimated state; replace signature as needed.
  float computeCommand(ControlData& state);

  // Set a new yaw reference and reset the integral term.
  void setYawReference(float yawDeg);

 private:
  static float wrapTo360(float deg);
  static float wrapErrorDeg(float actual360, float ref360);

  float yawRefDeg_ = 0.0f;   // stored in [0, 360)
  float yawRateRefDeg_ = 0.0f;
  float integralYawError_ = 0.0f;  // deg*s
};
