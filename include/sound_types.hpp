#pragma once

#include <string>
#include <unordered_map>

enum class SoundType {
  UNKNOWN,
  AIR_BRAKE,
  COUPLING,
  DECOUPLING,
  ENGINE_FAN,
  ENGINE_IDLE,
  ENGINE_REV,
  ENGINE_START,
  ENGINE_STOP,
  ENGINE_TURBO,
  GEAR_SHIFT,
  HORN,
  JAKE_BRAKE,
  PARKING_BRAKE,
  REVERSE,
  TURN_SIGNAL
};

inline SoundType getSoundTypeFromFilename(const std::string &filename) {
  std::string name = filename;
  // Remove extension if present
  size_t lastdot = name.find_last_of(".");
  if (lastdot != std::string::npos) {
    name = name.substr(0, lastdot);
  }
  
  // Remove leading path if present
  size_t lastslash = name.find_last_of("/");
  if (lastslash != std::string::npos) {
    name = name.substr(lastslash + 1);
  }

  if (name == "air_brake") return SoundType::AIR_BRAKE;
  if (name == "coupling") return SoundType::COUPLING;
  if (name == "decoupling") return SoundType::DECOUPLING;
  if (name == "engine_fan") return SoundType::ENGINE_FAN;
  if (name == "engine_idle") return SoundType::ENGINE_IDLE;
  if (name == "engine_rev") return SoundType::ENGINE_REV;
  if (name == "engine_start") return SoundType::ENGINE_START;
  if (name == "engine_stop") return SoundType::ENGINE_STOP;
  if (name == "engine_turbo") return SoundType::ENGINE_TURBO;
  if (name == "gear_shift") return SoundType::GEAR_SHIFT;
  if (name == "horn") return SoundType::HORN;
  if (name == "jake_brake") return SoundType::JAKE_BRAKE;
  if (name == "parking_brake") return SoundType::PARKING_BRAKE;
  if (name == "reverse") return SoundType::REVERSE;
  if (name == "turn_signal") return SoundType::TURN_SIGNAL;

  return SoundType::UNKNOWN;
}
