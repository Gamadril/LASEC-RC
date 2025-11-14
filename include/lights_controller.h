#pragma once

#include "RcEcuBus.h"
#include "RcEcuBusAddresses.h"
#include "board_config.h"
#include "esp_timer.h"

#define LIGHT_ON 0xFF
#define LIGHT_OFF 0x00

// Light intensity levels
#define LIGHT_DIM 0x40
#define LIGHT_NORMAL 0x80
#define LIGHT_BRIGHT 0xFF

class LightsController {
public:
  typedef void (*indicator_callback_t)(bool left, bool right);

  /**
   * Default constructor
   */
  LightsController() : _rcEcuBus(PIN_ECU_BUS_SDA, PIN_ECU_BUS_SCL) {
  }

  /**
   * Initialize the controller.
   *
   * @param indicator_callback - callback function that will be called each time the indicator LED
   * state changes.
   */
  void init(indicator_callback_t indicator_callback = NULL) {
    _indicator_callback = indicator_callback;
    _rcEcuBus.init();

    // Initialize known board addresses
    initBoards();

    esp_timer_create_args_t timer_args = {};
    timer_args.callback = (esp_timer_cb_t)&ticker_cb;
    timer_args.name = "lights_timer";
    timer_args.arg = this;
    timer_args.dispatch_method = ESP_TIMER_TASK;
    timer_args.skip_unhandled_events = false;

    esp_timer_create(&timer_args, &_timer);
    esp_timer_start_periodic(_timer, 375000); // 375ms in microseconds
  }

  /**
   * Sets the state of the left turn signal.
   * @param brightness - LED brightness (0 = off, >0 = on)
   */
  void setLeftIndicator(uint8_t brightness) {
    _leftIndicator.brightness = brightness;

    if (brightness > 0) {
      // Turn on left signal on specific boards and broadcast event
      turnOnLeftSignal();
    } else {
      // Turn off left signal
      turnOffLeftSignal();
    }
  }

  /**
   * Gets the actual state of the turn left signal.
   * @return true if on, false if off
   */
  bool isLeftIndicatorOn() {
    return _leftIndicator.on;
  }

  /**
   * Sets the state of the right turn signal.
   * @param brightness - LED brightness (0 = off, >0 = on)
   */
  void setRightIndicator(uint8_t brightness) {
    _rightIndicator.brightness = brightness;

    if (brightness > 0) {
      // Turn on right signal on specific boards and broadcast event
      turnOnRightSignal();
    } else {
      // Turn off right signal
      turnOffRightSignal();
    }
  }

  /**
   * Gets the actual state of the turn right signal.
   * @return true if on, false if off
   */
  bool isRightIndicatorOn() {
    return _rightIndicator.on;
  }

  /**
   * Initialize board addresses for direct control
   */
  void initBoards() {
    // Register known board addresses
    _rcEcuBus.registerBoard(FRONT_LEFT_LIGHTS_ADDRESS);
    _rcEcuBus.registerBoard(FRONT_RIGHT_LIGHTS_ADDRESS);
    _rcEcuBus.registerBoard(REAR_LEFT_LIGHTS_ADDRESS);
    _rcEcuBus.registerBoard(REAR_RIGHT_LIGHTS_ADDRESS);
    _rcEcuBus.registerBoard(ROOF_LIGHTS_ADDRESS);
  }

  /**
   * Sets the state of the low beam lights (multicast to all registered boards)
   * @param brightness - LED brightness
   */
  void setLowBeamLight(uint8_t brightness) {
    uint8_t data[2] = {EVENT_LOW_BEAM, brightness};
    _rcEcuBus.multicastCmd(CMD_DATA, data, 2);
  }

  /**
   * Sets the state of the high beam lights (multicast to all registered boards)
   * @param brightness - LED brightness
   */
  void setHighBeamLight(uint8_t brightness) {
    uint8_t data[2] = {EVENT_HIGH_BEAM, brightness};
    _rcEcuBus.multicastCmd(CMD_DATA, data, 2);
  }

  /**
   * Sets the state of the fog lights (multicast to all registered boards)
   * @param brightness - LED brightness
   */
  void setFogLight(uint8_t brightness) {
    uint8_t data[2] = {EVENT_FOG, brightness};
    _rcEcuBus.multicastCmd(CMD_DATA, data, 2);
  }

  /**
   * Sets the state of the daytime lights (multicast to all registered boards)
   * @param brightness - LED brightness
   */
  void setDaytimeLight(uint8_t brightness) {
    uint8_t data[2] = {EVENT_DAYTIME, brightness};
    _rcEcuBus.multicastCmd(CMD_DATA, data, 2);
  }

  /**
   * Sets the state of the front parking lights (front boards only)
   * @param brightness - LED brightness
   */
  void setParkingLight(uint8_t brightness) {
    uint8_t data[2] = {EVENT_PARKING, brightness};
    _rcEcuBus.multicastCmd(CMD_DATA, data, 2);
  }

  /**
   * Sets the state of the roof lights (roof board only)
   * @param brightness - LED brightness
   */
  void setRoofLight(uint8_t brightness) {
    uint8_t data[2] = {EVENT_PARKING, brightness};
    _rcEcuBus.sendCmd(CMD_DATA, data, 2, ROOF_LIGHTS_ADDRESS);
  }

  /**
   * Sets the state of the tail lights (rear boards only)
   * @param brightness - LED brightness
   */
  void setTailLight(uint8_t brightness) {
    uint8_t data[2] = {EVENT_PARKING, brightness};
    _rcEcuBus.multicastCmd(CMD_DATA, data, 2);
  }

  /**
   * Sets the state of the reversing lights (rear center board)
   * @param brightness - LED brightness
   */
  void setReversingLight(uint8_t brightness) {
    uint8_t data[2] = {EVENT_REVERSE, brightness};
    _rcEcuBus.sendCmd(CMD_DATA, data, 2, REAR_CENTER_LIGHTS_ADDRESS);
  }

  /**
   * Sets the state of the side lights (side boards only)
   * @param brightness - LED brightness
   */
  void setSideLight(uint8_t brightness) {
    uint8_t data[2] = {EVENT_SIDE_MARKER, brightness};
    _rcEcuBus.multicastCmd(CMD_DATA, data, 2);
  }

  /**
   * Sets the state of the cabin lights (interior board)
   * @param brightness - LED brightness
   */
  void setCabineLight(uint8_t brightness) {
    uint8_t data[2] = {EVENT_PARKING, brightness};
    _rcEcuBus.sendCmd(CMD_DATA, data, 2, INTERIOR_LIGHTS_ADDRESS);
  }

  /**
   * Sets the state of the brake lights (multicast to all registered boards)
   * @param brightness - LED brightness
   */
  void setBrakeLight(uint8_t brightness) {
    uint8_t data[2] = {EVENT_BRAKE, brightness};
    _rcEcuBus.multicastCmd(CMD_DATA, data, 2);
  }

  /**
   * Turn on left turn signal (specific boards + multicast)
   */
  void turnOnLeftSignal() {
    // Send to specific rear left board
    _rcEcuBus.sendCmd(EVENT_TURN_LEFT, REAR_LEFT_LIGHTS_ADDRESS);
    // Send to specific front left board
    _rcEcuBus.sendCmd(EVENT_TURN_LEFT, FRONT_LEFT_LIGHTS_ADDRESS);
    // Multicast the event so all boards know left turn is active
    _rcEcuBus.multicastCmd(EVENT_TURN_LEFT);
  }

  /**
   * Turn off left turn signal (specific boards + multicast)
   */
  void turnOffLeftSignal() {
    // Send to specific rear left board
    _rcEcuBus.sendCmd(EVENT_TURN_LEFT | 0x80, REAR_LEFT_LIGHTS_ADDRESS);
    // Send to specific front left board
    _rcEcuBus.sendCmd(EVENT_TURN_LEFT | 0x80, FRONT_LEFT_LIGHTS_ADDRESS);
    // Multicast the event so all boards know left turn is inactive
    _rcEcuBus.multicastCmd(EVENT_TURN_LEFT | 0x80);
  }

  /**
   * Turn on right turn signal (specific boards + multicast)
   */
  void turnOnRightSignal() {
    // Send to specific rear right board
    _rcEcuBus.sendCmd(EVENT_TURN_RIGHT, REAR_RIGHT_LIGHTS_ADDRESS);
    // Send to specific front right board
    _rcEcuBus.sendCmd(EVENT_TURN_RIGHT, FRONT_RIGHT_LIGHTS_ADDRESS);
    // Multicast the event so all boards know right turn is active
    _rcEcuBus.multicastCmd(EVENT_TURN_RIGHT);
  }

  /**
   * Turn off right turn signal (specific boards + multicast)
   */
  void turnOffRightSignal() {
    // Send to specific rear right board
    _rcEcuBus.sendCmd(EVENT_TURN_RIGHT | 0x80, REAR_RIGHT_LIGHTS_ADDRESS);
    // Send to specific front right board
    _rcEcuBus.sendCmd(EVENT_TURN_RIGHT | 0x80, FRONT_RIGHT_LIGHTS_ADDRESS);
    // Multicast the event so all boards know right turn is inactive
    _rcEcuBus.multicastCmd(EVENT_TURN_RIGHT | 0x80);
  }

  /**
   * Send a command to all registered light boards
   * @param command - Command to send
   */
  void sendToAllLights(uint8_t command) {
    _rcEcuBus.sendToAll(command);
  }

  /**
   * Send a command with data to all registered light boards
   * @param command - Command to send
   * @param data - Data to send
   * @param datalen - Size of the data
   */
  void sendToAllLights(uint8_t command, uint8_t *data, uint8_t datalen) {
    _rcEcuBus.sendToAll(command, data, datalen);
  }

  /**
   * Check how many light boards are responding
   * @return Number of responding boards
   */
  uint8_t pingAllLightBoards() {
    return _rcEcuBus.pingAllBoards();
  }

private:
  struct IndicatorState {
    bool on = false;
    // bool light_on = false;
    uint8_t brightness = LIGHT_OFF;
  };
  IndicatorState _leftIndicator;
  IndicatorState _rightIndicator;

  RcEcuBus _rcEcuBus;
  esp_timer_handle_t _timer;
  indicator_callback_t _indicator_callback = NULL;

  static void ticker_cb(void *arg) {
    LightsController *lc = (LightsController *)arg;
    if (lc->_leftIndicator.brightness == LIGHT_OFF) {
      lc->_leftIndicator.on = false;
    } else {
      lc->_leftIndicator.on = !lc->_leftIndicator.on;
    }

    if (lc->_rightIndicator.brightness == LIGHT_OFF) {
      lc->_rightIndicator.on = false;
    } else {
      lc->_rightIndicator.on = !lc->_rightIndicator.on;
    }

    if (lc->_indicator_callback != NULL && (lc->_leftIndicator.on || lc->_rightIndicator.on)) {
      lc->_indicator_callback(lc->_leftIndicator.on, lc->_rightIndicator.on);
    }
  }
};
