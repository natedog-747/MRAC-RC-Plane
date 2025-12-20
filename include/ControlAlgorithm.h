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
  void configureStepCommand(int steps, float stepDeg, float stepDurationSec);
  float yawReferenceDeg() const;

 private:
  static float wrapTo360(float deg);
  static float wrapErrorDeg(float actual360, float ref360);

  float yawRefDeg_ = 0.0f;   // stored in [0, 360)
  float yawRateRefDeg_ = 0.0f;
  float integralYawError_ = 0.0f;  // deg*s
  float baseYawRefDeg_ = 0.0f;     // reference at override entry before steps
  int totalSteps_ = 0;
  int completedSteps_ = 0;
  float stepSizeDeg_ = 0.0f;
  float stepDurationSec_ = 1.0f;
  float stepElapsedSec_ = 0.0f;
  bool stepHighPhase_ = false;     // false=base heading, true=base + stepSizeDeg_
};
