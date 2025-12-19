#include "CsvLogger.h"
#include <cstring>

CsvLogger::CsvLogger(uint8_t csPin, SPIClass& spi, const char* prefix)
    : csPin_(csPin), spi_(spi), prefix_(prefix) {}

bool CsvLogger::begin() {
  if (ready_) {
    return true;
  }

  spi_.begin();  // Ensure the bus is initialized (uses VSPI defaults when constructed with VSPI).
  if (!SD.begin(csPin_, spi_)) {
    if (!warned_) {
      Serial.println("SD card init failed; check wiring/CS pin.");
      warned_ = true;
    }
    ready_ = false;
    return false;
  }

  if (!chooseNewFileName()) {
    return false;
  }

  ready_ = true;
  warned_ = false;
  return true;
}

bool CsvLogger::append(const ControlData& data) {
  if (!begin()) {
    return false;
  }

  // If the file is new, add a header row before appending data.
  bool newFile = !SD.exists(filename_);
  File file = SD.open(filename_, FILE_APPEND);
  if (!file) {
    if (!warned_) {
      Serial.println("Failed to open CSV log for writing.");
      warned_ = true;
    }
    ready_ = false;
    return false;
  }

  if (newFile) {
    file.println("timestamp_ms,dt_sec,accel_x_mps2,accel_y_mps2,accel_z_mps2,gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,euler_roll_deg,euler_pitch_deg,euler_yaw_deg,euler_roll_rate_deg_s,euler_pitch_rate_deg_s,euler_yaw_rate_deg_s,control_signal,override_active");
  }

  file.printf("%lu,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
              static_cast<unsigned long>(data.timestampMs),
              static_cast<double>(data.dtSec),
              static_cast<double>(data.accelXMps2),
              static_cast<double>(data.accelYMps2),
              static_cast<double>(data.accelZMps2),
              static_cast<double>(data.gyroXRadPerSec),
              static_cast<double>(data.gyroYRadPerSec),
              static_cast<double>(data.gyroZRadPerSec),
              static_cast<double>(data.eulerRollDeg),
              static_cast<double>(data.eulerPitchDeg),
              static_cast<double>(data.eulerYawDeg),
              static_cast<double>(data.eulerRollRateDegPerSec),
              static_cast<double>(data.eulerPitchRateDegPerSec),
              static_cast<double>(data.eulerYawRateDegPerSec),
              static_cast<double>(data.controlSignal),
              data.overrideActive ? 1 : 0);
  file.close();
  return true;
}

bool CsvLogger::chooseNewFileName() {
  // Try log_000.csv .. log_999.csv; pick first not present.
  for (int i = 0; i <= 999; ++i) {
    snprintf(filename_, sizeof(filename_), "/%s_%03d.csv", prefix_, i);
    if (!SD.exists(filename_)) {
      return true;
    }
  }
  if (!warned_) {
    Serial.println("No available log filename (0-999).");
    warned_ = true;
  }
  return false;
}
