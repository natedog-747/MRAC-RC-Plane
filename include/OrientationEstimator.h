#pragma once

#include "ControlData.h"

// Skeleton orientation estimator; fill in with sensor fusion as needed.
class OrientationEstimator {
 public:
  OrientationEstimator() = default;

  // Update orientation estimate; replace parameters with real sensor inputs.
  ControlData estimate(uint32_t nowMs);
};
