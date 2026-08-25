#pragma once

#include "common.hpp"
#include "hal/persistence.hpp"
#include "vehicle.hpp"
#include "vehicles/actros_1851.hpp"

#include <cJSON.h>
#include <cstring>
#include <string>

#ifdef ESP_PLATFORM
#include "esp_log.h"
#include <cstdio>
#else
#include <iostream>
#endif

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
#define CH_HAZARD 9
#define CH9 8
#define CH10 9
#define CH11 10
#define CH12 11

class ConfigHandler {
public:
  ConfigHandler(Persistence &persistence, const std::string &path = "config.json")
      : _persistence(persistence), _path(path) {
  }

  void init() {
    _persistence.init(_path);

    std::string json;
    if (_persistence.load(json) && fromJson(json)) {
#ifdef ESP_PLATFORM
      ESP_LOGI(TAG, "Config loaded from %s", _path.c_str());
#else
      std::cout << "Config loaded from " << _path << std::endl;
#endif
    } else {
#ifdef ESP_PLATFORM
      ESP_LOGW(TAG, "No valid config at %s, writing defaults", _path.c_str());
#else
      std::cout << "No valid config at " << _path << ", writing defaults" << std::endl;
#endif
      _config = Config{};
      save();
    }
    print();
  }

  bool save() {
    std::string json = toJson(false);
    if (json.empty()) {
#ifdef ESP_PLATFORM
      ESP_LOGE(TAG, "Config serialize failed");
#endif
      return false;
    }
    if (!_persistence.save(json)) {
#ifdef ESP_PLATFORM
      ESP_LOGE(TAG, "Config save failed");
#endif
      return false;
    }
    return true;
  }

  void print() {
    std::string json = toJson(true);
    if (json.empty()) {
      return;
    }
#ifdef ESP_PLATFORM
    // printf: ESP_LOG truncates long lines (~1KB)
    ESP_LOGI(TAG, "=== Config (%s) ===", _path.c_str());
    printf("%s\n", json.c_str());
#else
    std::cout << "=== Config ===\n" << json << std::endl;
#endif
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
  static inline const char *TAG = "CFG";
  Config _config;
  Persistence &_persistence;
  std::string _path;

  static void setNumber(cJSON *obj, const char *key, double value) {
    cJSON_AddNumberToObject(obj, key, value);
  }

  static void setBool(cJSON *obj, const char *key, bool value) {
    cJSON_AddBoolToObject(obj, key, value);
  }

  static void setString(cJSON *obj, const char *key, const char *value) {
    cJSON_AddStringToObject(obj, key, value ? value : "");
  }

  // packed structs cannot bind fields to references
#define LOAD_NUM(obj, key, field)                                                                              \
  do {                                                                                                         \
    cJSON *_it = cJSON_GetObjectItemCaseSensitive((obj), (key));                                               \
    if (cJSON_IsNumber(_it)) {                                                                                 \
      (field) = static_cast<__typeof__(field)>(_it->valuedouble);                                              \
    }                                                                                                          \
  } while (0)

#define LOAD_BOOL(obj, key, field)                                                                             \
  do {                                                                                                         \
    cJSON *_it = cJSON_GetObjectItemCaseSensitive((obj), (key));                                               \
    if (cJSON_IsBool(_it)) {                                                                                   \
      (field) = cJSON_IsTrue(_it);                                                                             \
    }                                                                                                          \
  } while (0)

  static bool getString(cJSON *obj, const char *key, char *out, size_t outLen) {
    cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (!cJSON_IsString(item) || item->valuestring == nullptr) {
      return false;
    }
    strncpy(out, item->valuestring, outLen - 1);
    out[outLen - 1] = '\0';
    return true;
  }

  static cJSON *servoToJson(const ServoConfig &s) {
    cJSON *o = cJSON_CreateObject();
    setNumber(o, "min", s.min);
    setNumber(o, "neutral", s.neutral);
    setNumber(o, "max", s.max);
    setNumber(o, "frequency", s.frequency);
    return o;
  }

  static void servoFromJson(cJSON *o, ServoConfig &s) {
    if (!o) {
      return;
    }
    LOAD_NUM(o, "min", s.min);
    LOAD_NUM(o, "neutral", s.neutral);
    LOAD_NUM(o, "max", s.max);
    LOAD_NUM(o, "frequency", s.frequency);
  }

  static cJSON *escToJson(const EscConfig &e) {
    cJSON *o = cJSON_CreateObject();
    setNumber(o, "min", e.min);
    setNumber(o, "neutral", e.neutral);
    setNumber(o, "max", e.max);
    setNumber(o, "fwdStartGap", e.fwdStartGap);
    setNumber(o, "revStartGap", e.revStartGap);
    return o;
  }

  static void escFromJson(cJSON *o, EscConfig &e) {
    if (!o) {
      return;
    }
    LOAD_NUM(o, "min", e.min);
    LOAD_NUM(o, "neutral", e.neutral);
    LOAD_NUM(o, "max", e.max);
    LOAD_NUM(o, "fwdStartGap", e.fwdStartGap);
    LOAD_NUM(o, "revStartGap", e.revStartGap);
  }

  static cJSON *rcToJson(const RcConfig &r) {
    cJSON *o = cJSON_CreateObject();
    setString(o, "name", r.name);
    cJSON *map = cJSON_CreateArray();
    for (int i = 0; i < RC_MAX_CHANNELS; i++) {
      cJSON_AddItemToArray(map, cJSON_CreateNumber(r.channel_map[i]));
    }
    cJSON_AddItemToObject(o, "channel_map", map);
    return o;
  }

  static void rcFromJson(cJSON *o, RcConfig &r) {
    if (!o) {
      return;
    }
    getString(o, "name", r.name, RC_NAME_LEN);
    cJSON *map = cJSON_GetObjectItemCaseSensitive(o, "channel_map");
    if (cJSON_IsArray(map)) {
      int n = cJSON_GetArraySize(map);
      for (int i = 0; i < n && i < RC_MAX_CHANNELS; i++) {
        cJSON *item = cJSON_GetArrayItem(map, i);
        if (cJSON_IsNumber(item)) {
          r.channel_map[i] = static_cast<uint8_t>(item->valuedouble);
        }
      }
    }
  }

  static cJSON *vehicleToJson(const Vehicle &v) {
    cJSON *o = cJSON_CreateObject();
    setString(o, "name", v.name);
    setNumber(o, "startVolume", v.startVolume);
    setNumber(o, "idleVolume", v.idleVolume);
    setNumber(o, "engineIdleVolume", v.engineIdleVolume);
    setNumber(o, "fullThrottleVolume", v.fullThrottleVolume);
    setBool(o, "revSoundEnabled", v.revSoundEnabled);
    setNumber(o, "revVolume", v.revVolume);
    setNumber(o, "engineRevVolume", v.engineRevVolume);
    setNumber(o, "revSwitchPoint", v.revSwitchPoint);
    setNumber(o, "idleEndPoint", v.idleEndPoint);
    setNumber(o, "idleVolumeProportion", v.idleVolumeProportion);
    setBool(o, "jakeBrakeEnabled", v.jakeBrakeEnabled);
    setNumber(o, "jakeBrakeVolume", v.jakeBrakeVolume);
    setNumber(o, "jakeBrakeIdleVolume", v.jakeBrakeIdleVolume);
    setNumber(o, "jakeBrakeMinRpm", v.jakeBrakeMinRpm);
    setNumber(o, "dieselKnockVolume", v.dieselKnockVolume);
    setNumber(o, "dieselKnockIdleVolume", v.dieselKnockIdleVolume);
    setNumber(o, "dieselKnockInterval", v.dieselKnockInterval);
    setNumber(o, "dieselKnockStartPoint", v.dieselKnockStartPoint);
    setNumber(o, "dieselKnockAdaptiveVolume", v.dieselKnockAdaptiveVolume);
    setNumber(o, "dieselKnockPulses", v.dieselKnockPulses);
    setBool(o, "dieselKnockDependsOnRPM", v.dieselKnockDependsOnRPM);
    cJSON *cyl = cJSON_CreateArray();
    for (int i = 0; i < 4; i++) {
      cJSON_AddItemToArray(cyl, cJSON_CreateNumber(v.dieselKnockCylinders[i]));
    }
    cJSON_AddItemToObject(o, "dieselKnockCylinders", cyl);
    setBool(o, "turboEnabled", v.turboEnabled);
    setNumber(o, "turboVolume", v.turboVolume);
    setNumber(o, "turboIdleVolume", v.turboIdleVolume);
    setBool(o, "chargerEnabled", v.chargerEnabled);
    setNumber(o, "chargerVolume", v.chargerVolume);
    setNumber(o, "chargerIdleVolume", v.chargerIdleVolume);
    setNumber(o, "chargerStartPoint", v.chargerStartPoint);
    setBool(o, "wastegateEnabled", v.wastegateEnabled);
    setNumber(o, "wastegateVolume", v.wastegateVolume);
    setNumber(o, "wastegateIdleVolume", v.wastegateIdleVolume);
    setBool(o, "fanEnabled", v.fanEnabled);
    setNumber(o, "fanVolume", v.fanVolume);
    setNumber(o, "fanIdleVolume", v.fanIdleVolume);
    setNumber(o, "fanStartPoint", v.fanStartPoint);
    setNumber(o, "hornVolume", v.hornVolume);
    setNumber(o, "brakeVolume", v.brakeVolume);
    setNumber(o, "parkingBrakeVolume", v.parkingBrakeVolume);
    setNumber(o, "shiftingVolume", v.shiftingVolume);
    setNumber(o, "reversingVolume", v.reversingVolume);
    setNumber(o, "indicatorVolume", v.indicatorVolume);
    setNumber(o, "indicatorOnThreshold", v.indicatorOnThreshold);
    setBool(o, "couplingSoundEnabled", v.couplingSoundEnabled);
    setNumber(o, "couplingVolume", v.couplingVolume);
    setBool(o, "xenonLightsEnabled", v.xenonLightsEnabled);
    setBool(o, "separateHighBeamEnabled", v.separateHighBeamEnabled);
    setNumber(o, "escRampTimeFirstGear", v.escRampTimeFirstGear);
    setNumber(o, "escRampTimeSecondGear", v.escRampTimeSecondGear);
    setNumber(o, "escRampTimeThirdGear", v.escRampTimeThirdGear);
    setNumber(o, "escBrakeSteps", v.escBrakeSteps);
    setNumber(o, "escAccelerationSteps", v.escAccelerationSteps);
    setBool(o, "shiftingAutoThrottle", v.shiftingAutoThrottle);
    setNumber(o, "clutchEngagingPoint", v.clutchEngagingPoint);
    setNumber(o, "maxRpmPercentage", v.maxRpmPercentage);
    setNumber(o, "acc", v.acc);
    setNumber(o, "dec", v.dec);
    return o;
  }

  static void vehicleFromJson(cJSON *o, Vehicle &v) {
    if (!o) {
      return;
    }
    getString(o, "name", v.name, sizeof(v.name));
    LOAD_NUM(o, "startVolume", v.startVolume);
    LOAD_NUM(o, "idleVolume", v.idleVolume);
    LOAD_NUM(o, "engineIdleVolume", v.engineIdleVolume);
    LOAD_NUM(o, "fullThrottleVolume", v.fullThrottleVolume);
    LOAD_BOOL(o, "revSoundEnabled", v.revSoundEnabled);
    LOAD_NUM(o, "revVolume", v.revVolume);
    LOAD_NUM(o, "engineRevVolume", v.engineRevVolume);
    LOAD_NUM(o, "revSwitchPoint", v.revSwitchPoint);
    LOAD_NUM(o, "idleEndPoint", v.idleEndPoint);
    LOAD_NUM(o, "idleVolumeProportion", v.idleVolumeProportion);
    LOAD_BOOL(o, "jakeBrakeEnabled", v.jakeBrakeEnabled);
    LOAD_NUM(o, "jakeBrakeVolume", v.jakeBrakeVolume);
    LOAD_NUM(o, "jakeBrakeIdleVolume", v.jakeBrakeIdleVolume);
    LOAD_NUM(o, "jakeBrakeMinRpm", v.jakeBrakeMinRpm);
    LOAD_NUM(o, "dieselKnockVolume", v.dieselKnockVolume);
    LOAD_NUM(o, "dieselKnockIdleVolume", v.dieselKnockIdleVolume);
    LOAD_NUM(o, "dieselKnockInterval", v.dieselKnockInterval);
    LOAD_NUM(o, "dieselKnockStartPoint", v.dieselKnockStartPoint);
    LOAD_NUM(o, "dieselKnockAdaptiveVolume", v.dieselKnockAdaptiveVolume);
    LOAD_NUM(o, "dieselKnockPulses", v.dieselKnockPulses);
    LOAD_BOOL(o, "dieselKnockDependsOnRPM", v.dieselKnockDependsOnRPM);
    cJSON *cyl = cJSON_GetObjectItemCaseSensitive(o, "dieselKnockCylinders");
    if (cJSON_IsArray(cyl)) {
      int n = cJSON_GetArraySize(cyl);
      for (int i = 0; i < n && i < 4; i++) {
        cJSON *item = cJSON_GetArrayItem(cyl, i);
        if (cJSON_IsNumber(item)) {
          v.dieselKnockCylinders[i] = static_cast<uint8_t>(item->valuedouble);
        }
      }
    }
    LOAD_BOOL(o, "turboEnabled", v.turboEnabled);
    LOAD_NUM(o, "turboVolume", v.turboVolume);
    LOAD_NUM(o, "turboIdleVolume", v.turboIdleVolume);
    LOAD_BOOL(o, "chargerEnabled", v.chargerEnabled);
    LOAD_NUM(o, "chargerVolume", v.chargerVolume);
    LOAD_NUM(o, "chargerIdleVolume", v.chargerIdleVolume);
    LOAD_NUM(o, "chargerStartPoint", v.chargerStartPoint);
    LOAD_BOOL(o, "wastegateEnabled", v.wastegateEnabled);
    LOAD_NUM(o, "wastegateVolume", v.wastegateVolume);
    LOAD_NUM(o, "wastegateIdleVolume", v.wastegateIdleVolume);
    LOAD_BOOL(o, "fanEnabled", v.fanEnabled);
    LOAD_NUM(o, "fanVolume", v.fanVolume);
    LOAD_NUM(o, "fanIdleVolume", v.fanIdleVolume);
    LOAD_NUM(o, "fanStartPoint", v.fanStartPoint);
    LOAD_NUM(o, "hornVolume", v.hornVolume);
    LOAD_NUM(o, "brakeVolume", v.brakeVolume);
    LOAD_NUM(o, "parkingBrakeVolume", v.parkingBrakeVolume);
    LOAD_NUM(o, "shiftingVolume", v.shiftingVolume);
    LOAD_NUM(o, "reversingVolume", v.reversingVolume);
    LOAD_NUM(o, "indicatorVolume", v.indicatorVolume);
    LOAD_NUM(o, "indicatorOnThreshold", v.indicatorOnThreshold);
    LOAD_BOOL(o, "couplingSoundEnabled", v.couplingSoundEnabled);
    LOAD_NUM(o, "couplingVolume", v.couplingVolume);
    LOAD_BOOL(o, "xenonLightsEnabled", v.xenonLightsEnabled);
    LOAD_BOOL(o, "separateHighBeamEnabled", v.separateHighBeamEnabled);
    LOAD_NUM(o, "escRampTimeFirstGear", v.escRampTimeFirstGear);
    LOAD_NUM(o, "escRampTimeSecondGear", v.escRampTimeSecondGear);
    LOAD_NUM(o, "escRampTimeThirdGear", v.escRampTimeThirdGear);
    LOAD_NUM(o, "escBrakeSteps", v.escBrakeSteps);
    LOAD_NUM(o, "escAccelerationSteps", v.escAccelerationSteps);
    LOAD_BOOL(o, "shiftingAutoThrottle", v.shiftingAutoThrottle);
    LOAD_NUM(o, "clutchEngagingPoint", v.clutchEngagingPoint);
    LOAD_NUM(o, "maxRpmPercentage", v.maxRpmPercentage);
    LOAD_NUM(o, "acc", v.acc);
    LOAD_NUM(o, "dec", v.dec);
  }

public:
  std::string toJson(bool pretty = false) {
    cJSON *root = cJSON_CreateObject();
    if (!root) {
      return {};
    }

    setString(root, "model_name", _config.model_name);
    cJSON_AddItemToObject(root, "rcConfig", rcToJson(_config.rcConfig));
    cJSON_AddItemToObject(root, "steeringServo", servoToJson(_config.steeringServo));
    cJSON_AddItemToObject(root, "shiftingServo", servoToJson(_config.shiftingServo));
    cJSON_AddItemToObject(root, "escConfig", escToJson(_config.escConfig));
    setBool(root, "hasDashboard", _config.hasDashboard);
    setBool(root, "hasRgbLed", _config.hasRgbLed);
    setNumber(root, "volume", _config.volume);
    setNumber(root, "rgbColour", _config.rgbColour);
    setNumber(root, "rgbBrightness", _config.rgbBrightness);
    setBool(root, "autoTurnLights", _config.autoTurnLights);
    cJSON_AddItemToObject(root, "vehicle", vehicleToJson(_config.vehicle));

    char *printed = pretty ? cJSON_Print(root) : cJSON_PrintUnformatted(root);
    cJSON_Delete(root);
    if (!printed) {
      return {};
    }
    std::string out(printed);
    cJSON_free(printed);
    return out;
  }

  bool fromJson(const std::string &json) {
    cJSON *root = cJSON_Parse(json.c_str());
    if (!root) {
#ifdef ESP_PLATFORM
      const char *err = cJSON_GetErrorPtr();
      ESP_LOGE(TAG, "JSON parse error near: %s", err ? err : "?");
#endif
      return false;
    }

    // Start from defaults so missing fields keep defaults
    _config = Config{};

    getString(root, "model_name", _config.model_name, CONFIG_NAME_LEN);
    rcFromJson(cJSON_GetObjectItemCaseSensitive(root, "rcConfig"), _config.rcConfig);
    servoFromJson(cJSON_GetObjectItemCaseSensitive(root, "steeringServo"), _config.steeringServo);
    servoFromJson(cJSON_GetObjectItemCaseSensitive(root, "shiftingServo"), _config.shiftingServo);
    escFromJson(cJSON_GetObjectItemCaseSensitive(root, "escConfig"), _config.escConfig);
    LOAD_BOOL(root, "hasDashboard", _config.hasDashboard);
    LOAD_BOOL(root, "hasRgbLed", _config.hasRgbLed);
    LOAD_NUM(root, "volume", _config.volume);
    LOAD_NUM(root, "rgbColour", _config.rgbColour);
    LOAD_NUM(root, "rgbBrightness", _config.rgbBrightness);
    LOAD_BOOL(root, "autoTurnLights", _config.autoTurnLights);
    vehicleFromJson(cJSON_GetObjectItemCaseSensitive(root, "vehicle"), _config.vehicle);

    cJSON_Delete(root);
    return true;
  }
#undef LOAD_NUM
#undef LOAD_BOOL
};
