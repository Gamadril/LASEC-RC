#pragma once

#include "common.hpp"
#include "signal.hpp"
#include "utils.hpp"

class ModelState {
public:
  sigslot::signal<TurnSignal> onTurnIndicator;
  sigslot::signal<bool> onHazard;
  sigslot::signal<bool> onHorn;
  sigslot::signal<bool> onFailSafe;
  sigslot::signal<EngineState> onEngineState;
  sigslot::signal<uint16_t> onThrottle;
  sigslot::signal<uint8_t, uint8_t, bool> onGear;
  sigslot::signal<GearShift> onGearShift;
  sigslot::signal<uint16_t> onSteeringServo;
  sigslot::signal<uint16_t> onShiftingServo;

  ModelState() {
  }

  void setEngineState(EngineState state) {
    auto old = _state.engineState;
    _state.engineState = state;
    if (old != state) {
      onEngineState(state);
    }
  }

  EngineState getEngineState() {
    return _state.engineState;
  }

  void setDriveState(DriveState state) {
    _state.driveState = state;
  }

  DriveState getDriveState() {
    return _state.driveState;
  }

  void setHazard(bool value) {
    auto old = _state.hazard;
    _state.hazard = value;
    if (old != value) {
      onHazard(value);
    }
  }

  bool isHazard() {
    return _state.hazard;
  }

  void setHorn(bool value) {
    auto old = _state.horn;
    _state.horn = value;
    if (old != value) {
      onHorn(value);
    }
  }

  bool isHorn() {
    return _state.horn;
  }

  void setTurnSignal(TurnSignal value) {
    auto old = _state.turnSignal;
    _state.turnSignal = value;
    if (old != value) {
      onTurnIndicator(value);
    }
  }

  TurnSignal getTurnSignal() {
    return _state.turnSignal;
  }

  void setFailsafe(bool value) {
    auto old = _state.failsafe;
    _state.failsafe = value;
    if (old != value) {
      onFailSafe(value);
    }
  }

  bool isFailsafe() {
    return _state.failsafe;
  }

  void setClutch(bool value) {
    auto old = _state.clutch;
    _state.clutch = value;
    if (old != value) {
      onGear(_state.gear, _state.gear, value);
    }
  }

  bool isClutch() {
    return _state.clutch;
  }

  uint8_t getGear() {
    return _state.gear;
  }

  void setGear(uint8_t value) {
    auto old = _state.gear;
    _state.gear = value;
    if (old != value) {
      if (old < value) {
        setGearShift(UP);
      } else if (old > value) {
        setGearShift(DOWN);
      }
      onGear(old, value, _state.clutch);
    }
  }

  uint16_t getSpeed() {
    return _state.speed;
  }

  void setSpeed(uint16_t value) {
    _state.speed = value;
  }

  uint16_t getThrottle() {
    return _state.throttle;
  }

  void setThrottle(uint16_t value) {
    auto old = _state.throttle;
    _state.throttle = value;
    if (old != value) {
      onThrottle(value);
    }
  }

  void setThrottleRaw(uint16_t value) {
    _state.throttleRaw = value;
  }

  uint16_t getThrottleRaw() {
    return _state.throttleRaw;
  }

  void setJakeBrake(bool value) {
    _state.jakeBraking = value;
  }

  bool isJakeBrake() {
    return _state.jakeBraking;
  }

  void setAirBrake(bool value) {
    _state.airBrake = value;
  }

  bool isAirBrake() {
    return _state.airBrake;
  }

  void setWastegate(bool value) {
    _state.wastegate = value;
  }

  bool isWastegate() {
    return _state.wastegate;
  }

  void setGearShift(GearShift value) {
    auto old = _state.gearShift;
    _state.gearShift = value;
    if (old != value) {
      if (value == UP) {
        _stopShifting = millis() + 700;
      } else if (value == DOWN) {
        _stopShifting = millis() + 300;
      }
      onGearShift(value);
    }
  }

  GearShift getGearShift() {
    return _state.gearShift;
  }

  void checkGearShiftingStop() {
    if (millis() >= _stopShifting) {
      setGearShift(NOT);
    }
  }

  uint16_t getRPM() {
    return _state.rpm;
  }

  void setRPM(uint16_t value) {
    _state.rpm = constrain(value, 0, 500);
  }

  bool isBraking() {
    return _state.driveState == BRAKING_FORWARD || _state.driveState == BRAKING_BACKWARD;
  }

  bool isMovingBackward() {
    return _state.engineState == RUNNING &&
           (_state.driveState == DRIVING_BACKWARD || _state.driveState == BRAKING_BACKWARD);
  }

  bool isEngineRunningNotBreaking() {
    return isEngineRunning() && !isBraking();
  }

  bool isEngineRunning() {
    return _state.engineState == RUNNING;
  }

  bool isGearShifting() {
    return _state.gearShift == UP || _state.gearShift == DOWN;
  }

  void setChannelsCount(uint8_t value) {
    _state.channels_count = value;
  }

  void setChannelValues(uint16_t *values, uint8_t count) {
    memcpy(_state.channel_values, values, count * sizeof(uint16_t));
  }

  State *getState() {
    return &_state;
  }

private:
  State _state;
  int64_t _stopShifting;
};
