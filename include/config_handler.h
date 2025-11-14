#pragma once

#include "nvs.h"
#include "nvs_flash.h"
#include "vehicle.h"
#include "vehicles/MercedesActros1851.h"
#include <cstring>

#define SERIAL_BAUD_RATE 115200
#define CONFIG_NAME_LEN 40
#define RC_NAME_LEN 40
#define RC_MAX_CHANNELS 16

// CH1 THROTTLE
#define CH_THROTTLE 1
// CH2 STEERING
#define CH_STEERING 4
// CH3 SHIFTING
#define CH_SHIFTING 5
// CH4 HORN
#define CH_HORN 7
// CH5 RGB
#define CH_RGB 8
// CH6 GEAR NEUTRAL
#define CH_GEAR_CLUTCH 6
// CH7 MODE SWITCH
#define CH_MODE 10
// CH8 LIGHTS
#define CH_LIGHT 9
#define CH9 8
#define CH10 9
#define CH11 10
#define CH12 11

struct __attribute__((packed)) ServoConfig {
  uint16_t min = 1000;
  uint16_t neutral = 1500;
  uint16_t max = 2000;
  uint8_t frequency = 100; // 50 Hz - analog servos, 100 Hz - digital servos
};

struct __attribute__((packed)) EscConfig {
  uint16_t min = 1000;
  uint16_t neutral = 1500;
  uint16_t max = 2000;
};

struct __attribute__((packed)) RcConfig {
  char name[RC_NAME_LEN];
  uint8_t channel_map[RC_MAX_CHANNELS];
};

struct __attribute__((packed)) Config {
  char model_name[CONFIG_NAME_LEN] = {'M', 'A', 'I', 'N'};
  RcConfig rcConfig;
  ServoConfig steeringServo;
  ServoConfig shiftingServo;
  EscConfig escConfig;
  bool hasDashboard = false;
  bool hasRgbLed = false;
  uint8_t volume = 100;
  uint32_t rgbColour;
  uint8_t rgbBrightness;
  bool autoTurnLights = true;
  Vehicle vehicle = ACTROS_1851;
};

class ConfigHandler {
private:
  nvs_handle_t nvs_handle;

public:
  void init() {
    // Try to initialize NVS, but don't hang if it fails
    esp_err_t err = nvs_open("LASEC", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK) {
      size_t required_size = sizeof(Config);
      err = nvs_get_blob(nvs_handle, "config", &_config, &required_size);
      if (err != ESP_OK) {
        printf("NVS read failed: %s\n", esp_err_to_name(err));
      }
      nvs_close(nvs_handle);
    } else {
      printf("NVS initialization failed: %s, using default config\n", esp_err_to_name(err));
    }
  }

  void save() {
    esp_err_t err = nvs_open("LASE", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
      err = nvs_set_blob(nvs_handle, "config", &_config, sizeof(Config));
      if (err == ESP_OK) {
        nvs_commit(nvs_handle);
      } else {
        printf("NVS save failed: %s\n", esp_err_to_name(err));
      }
      nvs_close(nvs_handle);
    } else {
      printf("NVS save failed: %s\n", esp_err_to_name(err));
    }
  }

  Config *getConfig() {
    return &_config;
  }

  void setConfig(Config *config) {
    memcpy(&_config, config, sizeof(Config));
  }

  char *getModelName() {
    return _config.model_name;
  }

  bool hasDashboard() {
    return _config.hasDashboard;
  }

  RcConfig *getRcConfig() {
    return &(_config.rcConfig);
  }

  void setRcConfig(RcConfig *config) {
    memcpy(&(_config.rcConfig), config, sizeof(RcConfig));
  }

  ServoConfig *getSteeringServoConfig() {
    return &(_config.steeringServo);
  }

  ServoConfig *getShiftingServoConfig() {
    return &(_config.shiftingServo);
  }

  Vehicle *getVehicle() {
    return &(_config.vehicle);
  }

  bool hasRgbLed() {
    return _config.hasRgbLed;
  }

  uint32_t getRGBColour() {
    return _config.rgbColour;
  }

  void setRGBColour(uint32_t colour) {
    _config.rgbColour = colour;
  }

  uint8_t getRGBBrightness() {
    return _config.rgbBrightness;
  }

  void setRGBBrightness(uint8_t brightness) {
    _config.rgbBrightness = brightness;
  }

  uint8_t getSteeringFrequency() {
    return _config.steeringServo.frequency;
  }

  uint8_t getShiftingFrequency() {
    return _config.shiftingServo.frequency;
  }

  uint8_t getVolume() {
    return _config.volume;
  }

private:
  Config _config;
};
