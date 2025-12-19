#include <Arduino.h>
#include "OrientationEstimator.h"

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

  sensors_event_t orientationEvent;
  bno_.getEvent(&orientationEvent, Adafruit_BNO055::VECTOR_EULER);

  data.rollDeg = orientationEvent.orientation.x;
  data.pitchDeg = orientationEvent.orientation.y;
  data.yawDeg = orientationEvent.orientation.z;
  return data;
}
