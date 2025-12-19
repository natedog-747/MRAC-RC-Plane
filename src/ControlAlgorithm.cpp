#include "ControlAlgorithm.h"

float ControlAlgorithm::computeCommand(const ControlData& state) {
  (void)state; // unused placeholder
  // TODO: implement control logic; default to neutral command (90 deg servo).
  return 90.0f;
}
