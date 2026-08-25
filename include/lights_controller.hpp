#pragma once

#include "RcEcuBus.hpp"
#include "board_config.hpp"
#include "hal/timer.hpp"
#include <functional>

class LightsController {
public:
  using TimerFactory = std::function<Timer *()>;

  /**
   * Default constructor
   */
  LightsController(TimerFactory factory) : _rcEcuBus(PIN_ECU_BUS_SDA, PIN_ECU_BUS_SCL), _turnSignalTimer(factory()) {
  }

  /**
   * Initialize the controller.
   */
  void init() {
    _rcEcuBus.init();
    _turnSignalTimer->onAlarm.connect(&LightsController::onTimerAlarm, this);
  }

  /**
   * Initialize board addresses for direct control
   */
  void initBoards(const uint8_t board_adresses[], uint8_t nr_boards) {
    for (size_t i = 0; i < nr_boards; i++) {
      _rcEcuBus.registerBoard(board_adresses[i]);
    }
  }

  void setLowBeam(uint8_t state) {
    uint8_t data[2] = {EVENT_LOW_BEAM, state};
    _rcEcuBus.sendToAll(CMD_DATA, data, 2);
  }

  void setHighBeam(uint8_t state) {
    uint8_t data[2] = {EVENT_HIGH_BEAM, state};
    _rcEcuBus.sendToAll(CMD_DATA, data, 2);
  }

  void setFog(uint8_t state) {
    uint8_t data[2] = {EVENT_FOG, state};
    _rcEcuBus.sendToAll(CMD_DATA, data, 2);
  }

  void setDaytime(uint8_t state) {
    uint8_t data[2] = {EVENT_DAYTIME, state};
    _rcEcuBus.sendToAll(CMD_DATA, data, 2);
  }

  void setParking(uint8_t state) {
    uint8_t data[2] = {EVENT_PARKING, state};
    _rcEcuBus.sendToAll(CMD_DATA, data, 2);
  }

  void setReversing(uint8_t state) {
    uint8_t data[2] = {EVENT_REVERSE, state};
    _rcEcuBus.sendToAll(CMD_DATA, data, 2);
  }

  void setSideMarker(uint8_t state) {
    uint8_t data[2] = {EVENT_PARKING, state};
    _rcEcuBus.sendToAll(CMD_DATA, data, 2);
  }

  void setBrake(uint8_t state) {
    uint8_t data[2] = {EVENT_BRAKE, state};
    _rcEcuBus.sendToAll(CMD_DATA, data, 2);
  }

  void setHazard(bool state) {
    _hazard = state;
    checkTimer();
  }

  void setTurnLeft(bool state) {
    _turnLeft = state;
    checkTimer();
  }

  void setTurnRight(bool state) {
    _turnRight = state;
    checkTimer();
  }

  /**
   * Send a command to all registered light boards
   * @param command - Command to send
   */
  void sendToAll(uint8_t command) {
    _rcEcuBus.sendToAll(command);
  }

  /**
   * Send a command with data to all registered light boards
   * @param command - Command to send
   * @param data - Data to send
   * @param datalen - Size of the data
   */
  void sendToAll(uint8_t command, uint8_t *data, uint8_t datalen) {
    _rcEcuBus.sendToAll(command, data, datalen);
  }

  /**
   * Check how many light boards are responding
   * @return Number of responding boards
   */
  uint8_t pingAllBoards() {
    return _rcEcuBus.pingAllBoards();
  }

protected:
  void onTimerAlarm() {
    _blinkState = !_blinkState;
    if (_turnLeft || _hazard) {
      sendTurnLeft(_blinkState ? LIGHT_ON : LIGHT_OFF);
    }
    if (_turnRight || _hazard) {
      sendTurnRight(_blinkState ? LIGHT_ON : LIGHT_OFF);
    }
  }

private:
  void sendTurnLeft(uint8_t state) {
    uint8_t data[2] = {EVENT_TURN_LEFT, state};
    _rcEcuBus.sendToAll(CMD_DATA, data, 2);
  }

  void sendTurnRight(uint8_t state) {
    uint8_t data[2] = {EVENT_TURN_RIGHT, state};
    _rcEcuBus.sendToAll(CMD_DATA, data, 2);
  }

  void checkTimer() {
    if (_turnSignalTimer->is_running() && !_turnLeft && !_turnRight && !_hazard) {
      _turnSignalTimer->stop();
    } else if (!_turnSignalTimer->is_running() && (_turnLeft || _turnRight || _hazard)) {
      _turnSignalTimer->start();
    }
  }

  RcEcuBus _rcEcuBus;
  TimerFactory _timerFactory;
  Timer *_turnSignalTimer = nullptr;
  bool _hazard = false;
  bool _turnLeft = false;
  bool _turnRight = false;
  bool _blinkState = false;
};
