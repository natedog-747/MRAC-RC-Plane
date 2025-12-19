#pragma once

#include "ControlData.h"

// Skeleton control algorithm; fill in with your control logic.
class ControlAlgorithm {
 public:
  ControlAlgorithm() = default;

  // Compute actuator command based on estimated state; replace signature as needed.
  float computeCommand(const ControlData& state);
};
