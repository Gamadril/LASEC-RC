#pragma once

#include <cstdint>

#include "vehicle.hpp"
#include "vehicles/actros_1851.hpp"

#define CONFIG_NAME_LEN 40
#define RC_NAME_LEN 40
#define RC_MAX_CHANNELS 16

#define MIN_THROTTLE 0
#define MAX_THROTTLE 500

#define MIN_SPEED 0
#define MAX_SPEED 500

enum EngineState { OFF, STARTING, RUNNING, STOPPING, PARKING_BRAKE };

enum DriveState { STANDING, DRIVING_FORWARD, BRAKING_FORWARD, DRIVING_BACKWARD, BRAKING_BACKWARD };

enum GearShift { NOT, DOWN, UP };

enum TurnSignal { T_OFF, T_LEFT, T_RIGHT };

struct __attribute__((packed)) ServoConfig {
  uint16_t min = 1000;
  uint16_t neutral = 1500;
  uint16_t max = 2000;
  uint8_t frequency = 50; // 50 Hz - analog servo, 100 Hz - digital servo
};

struct __attribute__((packed)) EscConfig {
  uint16_t min = 1000;
  uint16_t neutral = 1500;
  uint16_t max = 2000;
  uint8_t fwdStartGap = 0;
  uint8_t revStartGap = 0;
};

struct __attribute__((packed)) RcConfig {
  char name[RC_NAME_LEN];
  uint8_t channel_map[RC_MAX_CHANNELS];
};

struct __attribute__((packed)) Config {
  char model_name[CONFIG_NAME_LEN] = "LASEC-RC";
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

struct __attribute__((packed)) State {
  bool hazard = false;
  bool horn = false;
  bool failsafe = false;
  bool jakeBraking = false;
  bool airBrake = false;
  bool parkingBrake = false;
  bool wastegate = false;
  bool clutch = false;
  GearShift gearShift = NOT;
  uint16_t throttle = 0;
  uint16_t throttleRaw = 0;
  uint16_t rpm = 0;
  uint16_t speed = 0;
  uint8_t gear = 0;
  EngineState engineState = OFF;
  DriveState driveState = STANDING;
  TurnSignal turnSignal = T_OFF;
  uint16_t channel_values[RC_MAX_CHANNELS] = {0};
  uint8_t channels_count = 0;
};