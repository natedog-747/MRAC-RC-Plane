#include <Arduino.h>
#include "ControlServoTask.h"
#include "CounterTask.h"

// GPIO configuration for the RC channel mirror
static const int PWM_IN_PIN  = 12;  // GPIO with Lemon RX channel input
static const int PWM_OUT_PIN = 14;  // GPIO to output the mirrored signal
static const int PWM_OVERRIDE_PIN = 27; // GPIO with override input (e.g. separate channel)

// Servo output limits (microseconds)
static const uint16_t SERVO_MIN_US = 1000;
static const uint16_t SERVO_MAX_US = 2000;

// Override behavior: when override pulse >= threshold, drive servo to override angle instead of mirroring
static const uint32_t OVERRIDE_THRESHOLD_US = 1500; // near servo midpoint
static const int OVERRIDE_ANGLE_DEG = 160;          // angle to drive when override is active

// Instantiate tasks with the chosen pins/config
ControlServoTask controlServoTask(PWM_IN_PIN,
                                  PWM_OUT_PIN,
                                  PWM_OVERRIDE_PIN,
                                  OVERRIDE_THRESHOLD_US,
                                  OVERRIDE_ANGLE_DEG,
                                  SERVO_MIN_US,
                                  SERVO_MAX_US);
CounterTask counterTask(controlServoTask.overrideFlag());

void setup() {
  Serial.begin(9600);
  // while (!Serial) { vTaskDelay(1); } // optional wait for USB

  controlServoTask.start();
  counterTask.start();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000)); // idle; tasks do the work
}
