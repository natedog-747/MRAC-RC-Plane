#include "SdLoggerTask.h"
#include <SPI.h>
#include "CsvLogger.h"

namespace {
// VSPI default pins for the ESP32 DOIT devkit; adjust if your wiring differs.
constexpr int SD_SCLK_PIN = 18;
constexpr int SD_MISO_PIN = 19;
constexpr int SD_MOSI_PIN = 23;
constexpr int SD_CS_PIN   = 5;
}  // namespace

SdLoggerTask::SdLoggerTask(QueueHandle_t dataQueue, BaseType_t core)
    : TaskBase("SdLogger", 4096, 1, core), dataQueue_(dataQueue) {}

void SdLoggerTask::setQueue(QueueHandle_t dataQueue) {
  dataQueue_ = dataQueue;
}

void SdLoggerTask::run() {
  SPIClass vspi(VSPI);
  vspi.begin(SD_SCLK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  CsvLogger csvLogger(SD_CS_PIN, vspi);
  bool sdReady = csvLogger.begin();
  bool sdWarned = false;

  ControlData incoming;
  for (;;) {
    if (dataQueue_ && xQueueReceive(dataQueue_, &incoming, pdMS_TO_TICKS(100)) == pdPASS) {
      // Mirror to Serial for quick visibility. When override is off, print orientation only.
      if (incoming.overrideActive) {
        Serial.printf("[LOG] t=%lu dt=%.4f accel(m/s^2)=(%.3f,%.3f,%.3f) gyro(rad/s)=(%.3f,%.3f,%.3f) euler(deg)=(%.2f,%.2f,%.2f) cmd=%.2f override=%d\n",
                      static_cast<unsigned long>(incoming.timestampMs),
                      static_cast<double>(incoming.dtSec),
                      static_cast<double>(incoming.accelXMps2),
                      static_cast<double>(incoming.accelYMps2),
                      static_cast<double>(incoming.accelZMps2),
                      static_cast<double>(incoming.gyroXRadPerSec),
                      static_cast<double>(incoming.gyroYRadPerSec),
                      static_cast<double>(incoming.gyroZRadPerSec),
                      static_cast<double>(incoming.eulerRollDeg),
                      static_cast<double>(incoming.eulerPitchDeg),
                      static_cast<double>(incoming.eulerYawDeg),
                      static_cast<double>(incoming.controlSignal),
                      incoming.overrideActive ? 1 : 0);
      } else {
        Serial.printf("[LOG] t=%lu dt=%.4f accel(m/s^2)=(%.3f,%.3f,%.3f) gyro(rad/s)=(%.3f,%.3f,%.3f) euler(deg)=(%.2f,%.2f,%.2f) euler_rate(deg/s)=(%.2f,%.2f,%.2f)\n",
                      static_cast<unsigned long>(incoming.timestampMs),
                      static_cast<double>(incoming.dtSec),
                      static_cast<double>(incoming.accelXMps2),
                      static_cast<double>(incoming.accelYMps2),
                      static_cast<double>(incoming.accelZMps2),
                      static_cast<double>(incoming.gyroXRadPerSec),
                      static_cast<double>(incoming.gyroYRadPerSec),
                      static_cast<double>(incoming.gyroZRadPerSec),
                      static_cast<double>(incoming.eulerRollDeg),
                      static_cast<double>(incoming.eulerPitchDeg),
                      static_cast<double>(incoming.eulerYawDeg),
                      static_cast<double>(incoming.eulerRollRateDegPerSec),
                      static_cast<double>(incoming.eulerPitchRateDegPerSec),
                      static_cast<double>(incoming.eulerYawRateDegPerSec));
      }

      if (!sdReady) {
        sdReady = csvLogger.begin();
        if (sdReady) {
          sdWarned = false;
        }
      }

      if (sdReady && !csvLogger.append(incoming)) {
        sdReady = false;  // Force re-init on the next loop.
        if (!sdWarned) {
          Serial.println("SD write failed; will retry initialization.");
          sdWarned = true;
        }
      }
    }
  }
}
