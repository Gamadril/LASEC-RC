#pragma once

enum EngineState { OFF, STARTING, RUNNING, STOPPING, PARKING_BRAKE };

enum DriveState { STANDING, DRIVING_FORWARD, BRAKING_FORWARD, DRIVING_BACKWARD, BRAKING_BACKWARD };

enum GearShift { NOT, DOWN, UP };

enum Mode { CONFIG, PLAY };

enum TurnSignal { T_OFF, T_LEFT, T_RIGHT };

struct __attribute__((packed)) State {
  bool hazard = false;
  bool failsafe = false;
  bool jakeBraking = false;
  bool airBrake = false;
  bool parkingBrake = false;
  bool wastegate = false;
  GearShift gearShift = NOT;
  uint16_t throttle = 0;
  uint16_t rpm = 0;
  uint16_t speed = 0;
  uint8_t gear = 0;
  EngineState engineState = OFF;
  DriveState driveState = STANDING;
  TurnSignal turnSignal = T_OFF;
};

class ModelState {
public:
  ModelState() {
  }

  void setEngineState(EngineState state) {
    _state.engineState = state;
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
    _state.hazard = value;
  }

  bool isHazard() {
    return _state.hazard;
  }

  void setTurnSignal(TurnSignal value) {
    _state.turnSignal = value;
  }

  TurnSignal getTurnSignal() {
    return _state.turnSignal;
  }

  void setFailsafe(bool value) {
    _state.failsafe = value;
    _state.hazard = value;
  }

  bool isFailsafe() {
    return _state.failsafe;
  }

  void setMode(Mode mode) {
    _mode = mode;
  }

  Mode getMode() {
    return _mode;
  }

  uint8_t getGear() {
    return _state.gear;
  }

  void setGear(uint8_t value) {
    _state.gear = value;
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
    _state.throttle = constrain(value, 0, 500);
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
    _state.gearShift = value;
  }

  GearShift getGearShift() {
    return _state.gearShift;
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

  bool debug() {
    printf("Hazard: %d\nGearShift: %d\nThrottle %d\nRPM: %d\nSpeed: %d\nGear: "
           "%d\nEngineState: %d\nDriveState: %d\n\n",
           _state.hazard, _state.gearShift, _state.throttle, _state.rpm, _state.speed, _state.gear,
           _state.engineState, _state.driveState);
    return true;
  }

  State *getState() {
    return &_state;
  }

private:
  State _state;
  Mode _mode = PLAY;
};
