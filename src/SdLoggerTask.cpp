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
      Serial.printf("euler(deg) roll=%.2f pitch=%.2f yaw=%.2f | yaw_ref=%.2f\n",
                    static_cast<double>(incoming.eulerRollDeg),
                    static_cast<double>(incoming.eulerPitchDeg),
                    static_cast<double>(incoming.eulerYawDeg),
                    static_cast<double>(incoming.yawRefDeg));

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
