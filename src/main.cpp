#include <Arduino.h>
#include "freertos/FreeRTOS.h"

TaskHandle_t core0TaskHandle = nullptr;
TaskHandle_t core1TaskHandle = nullptr;

void core0Task(void *pvParameters) {
  pinMode(LED_BUILTIN, OUTPUT);
  bool on = false;
  for (;;) {
    digitalWrite(LED_BUILTIN, on ? HIGH : LOW);
    on = !on;
    vTaskDelay(pdMS_TO_TICKS(500));  // 0.5s blink
  }
}

void core1Task(void *pvParameters) {
  int counter = 1;
  for (;;) {
    Serial.printf("Count: %d\n", counter++);
    vTaskDelay(pdMS_TO_TICKS(1000)); // 1s interval
  }
}

void setup() {
  Serial.begin(9600);
  // while (!Serial) { vTaskDelay(1); } // optional wait for USB

  xTaskCreatePinnedToCore(core0Task, "Core0Blink", 2048, nullptr, 1, &core0TaskHandle, 0);
  xTaskCreatePinnedToCore(core1Task, "Core1Count", 2048, nullptr, 1, &core1TaskHandle, 1);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));   // keep loop idle; tasks do the work
}
