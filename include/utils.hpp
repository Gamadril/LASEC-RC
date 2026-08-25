#pragma once

#include <cstdint>

#include "common.hpp"

#if defined(ESP_PLATFORM)
#include "esp_timer.h"
#else
#include <chrono>
#endif

// Constrain function (if not defined)
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

#if !defined(ESP_PLATFORM)
#define ESP_LOGI(tag, format, ...) fprintf(stderr, "INFO [%s]: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGE(tag, format, ...) fprintf(stderr, "ERROR [%s]: " format "\n", tag, ##__VA_ARGS__)
#define ESP_LOGW(tag, format, ...) fprintf(stderr, "WARN [%s]: " format "\n", tag, ##__VA_ARGS__)
#endif

// Audio sample structure
struct AudioSample {
  int16_t left;
  int16_t right;
};

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  const long dividend = out_max - out_min;
  const long divisor = in_max - in_min;
  const long delta = x - in_min;
  if (divisor == 0) {
    ESP_LOGE("UTL", "Invalid map input range, min == max");
    return -1;
  }
  return (delta * dividend + (divisor / 2)) / divisor + out_min;
}

std::string ds2str(DriveState state) {
  std::string str;

  switch (state) {
    case STANDING:
      str = "STANDING";
      break;
    case DRIVING_FORWARD:
      str = "DRIVING_FORWARD";
      break;
    case DRIVING_BACKWARD:
      str = "DRIVING_BACKWARD";
      break;
    case BRAKING_FORWARD:
      str = "BRAKING_FORWARD";
      break;
    case BRAKING_BACKWARD:
      str = "BRAKING_BACKWARD";
      break;
    default:
      str = "UNKNONWN";
      break;
  }
  return str;
}

std::string es2str(EngineState state) {
  std::string str;

  switch (state) {
    case OFF:
      str = "OFF";
      break;
    case STARTING:
      str = "STARTING";
      break;
    case RUNNING:
      str = "RUNNING";
      break;
    case STOPPING:
      str = "STOPPING";
      break;
    case PARKING_BRAKE:
      str = "PARKING_BRAKE";
      break;
    default:
      str = "UNKNONWN";
      break;
  }
  return str;
}

#if !defined(ESP_PLATFORM)
// Read little-endian 16-bit value
template <typename FileType> int16_t readInt16LE(FileType &file) {
  uint8_t bytes[2];
  file.read(reinterpret_cast<char *>(bytes), 2);
  return static_cast<int16_t>(bytes[0] | (bytes[1] << 8));
}

// Read little-endian 32-bit value
template <typename FileType> uint32_t readUint32LE(FileType &file) {
  uint8_t bytes[4];
  file.read(reinterpret_cast<char *>(bytes), 4);
  return bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
}

// Read little-endian 16-bit value
template <typename FileType> uint16_t readUint16LE(FileType &file) {
  uint8_t bytes[2];
  file.read(reinterpret_cast<char *>(bytes), 2);
  return bytes[0] | (bytes[1] << 8);
}

// Time functions
int64_t millis() {
  static auto start = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

int64_t micros() {
  static auto start = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
}

#else
int64_t IRAM_ATTR micros() {
  return esp_timer_get_time();
}

int64_t IRAM_ATTR millis() {
  return esp_timer_get_time() / 1000;
}
#endif