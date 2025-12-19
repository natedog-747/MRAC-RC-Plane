#pragma once

#include <Arduino.h>
#include <SD.h>
#include <SPI.h>
#include "ControlData.h"

// Simple CSV logger that mirrors Serial output to an SD card.
class CsvLogger {
 public:
  CsvLogger(uint8_t csPin, SPIClass& spi, const char* prefix = "log");

  // Initialize SD card + file; safe to call repeatedly.
  bool begin();

  // Append one row to the CSV file.
  bool append(const ControlData& data);

 private:
  bool chooseNewFileName();
  bool ensureFileReady();
  bool writeHeaderIfNeeded(File& file);

  uint8_t csPin_;
  SPIClass& spi_;
  const char* prefix_;
  char filename_[24] = {0};  // e.g., /log_001.csv
  bool ready_ = false;
  bool warned_ = false;
};
