#include "OrientationEstimator.h"

ControlData OrientationEstimator::estimate(uint32_t nowMs) {
  ControlData data;
  data.timestampMs = nowMs;
  // TODO: replace with real sensor fusion; placeholder keeps zeros.
  return data;
}
