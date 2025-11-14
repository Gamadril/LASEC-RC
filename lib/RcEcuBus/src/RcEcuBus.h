#pragma once

#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "RcEcuBusCommands.h"

#define I2C_MASTER_FREQ_HZ 100000
#define MAX_BOARDS 8
#define I2C_TIMEOUT_MS 1000

class RcEcuBus {
public:
  /**
   * Default constructor
   */
  RcEcuBus(int sda_pin, int scl_pin) {
    _sda = sda_pin;
    _scl = scl_pin;
  }

  /**
   * Init communication
   */
  void init() {
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = _sda;
    conf.scl_io_num = _scl;
    conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
    conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;

    ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0));
  }

  /**
   * Deinit communication
   */
  void end() {
  }

  /**
   * Register a board address for targeted communication
   * @param address - I2C address of the board
   */
  void registerBoard(uint8_t address) {
    if (_boardCount < MAX_BOARDS && !isBoardRegistered(address)) {
      _registeredBoards[_boardCount++] = address;
    }
  }

  /**
   * Check if a board address is registered
   * @param address - I2C address to check
   * @return true if registered
   */
  bool isBoardRegistered(uint8_t address) {
    for (int i = 0; i < _boardCount; i++) {
      if (_registeredBoards[i] == address) {
        return true;
      }
    }
    return false;
  }

  /**
   * Send a command with data to all registered clients (multicast)
   * @param command - Command to send
   * @param data - Data to send
   * @param datalen - Size of the data to send
   */
  void multicastCmd(uint8_t command, uint8_t *data, uint8_t datalen) {
    for (int i = 0; i < _boardCount; i++) {
      sendCmd(command, data, datalen, _registeredBoards[i]);
    }
  }

  /**
   * Send a command without data to all registered clients (multicast)
   * @param command - Command to send
   */
  void multicastCmd(uint8_t command) {
    for (int i = 0; i < _boardCount; i++) {
      sendCmd(command, _registeredBoards[i]);
    }
  }

  /**
   * Send a command with data to all registered clients (alternative name)
   * @param command - Command to send
   * @param data - Data to send
   * @param datalen - Size of the data to send
   */
  void sendToAll(uint8_t command, uint8_t *data, uint8_t datalen) {
    multicastCmd(command, data, datalen);
  }

  /**
   * Send a command without data to all registered clients (alternative name)
   * @param command - Command to send
   */
  void sendToAll(uint8_t command) {
    multicastCmd(command);
  }

  /**
   * Get the number of registered boards
   * @return Number of registered boards
   */
  uint8_t getRegisteredBoardCount() {
    return _boardCount;
  }

  /**
   * Check if no boards are registered
   * @return true if no boards registered
   */
  bool isEmpty() {
    return _boardCount == 0;
  }

  /**
   * Clear all registered boards
   */
  void clearRegisteredBoards() {
    _boardCount = 0;
  }

  /**
   * Send ping command to all registered boards
   * @return Number of boards that responded
   */
  uint8_t pingAllBoards() {
    uint8_t respondingBoards = 0;
    for (int i = 0; i < _boardCount; i++) {
      uint16_t response = readData(CMD_PING, _registeredBoards[i]);
      if (response != 0xFFFF) {
        respondingBoards++;
      }
    }
    return respondingBoards;
  }

  /**
   * Send a command with data to the specified client
   * @param command - Command to send
   * @param data - Data to send
   * @param datalen - Size of the data to send
   * @param address - address of the client
   */
  void sendCmd(uint8_t command, uint8_t *data, uint8_t datalen, uint8_t address) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // Regular I2C address (7-bit)
    i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, command, true);
    if (data != nullptr && datalen > 0) {
      i2c_master_write(cmd, data, datalen, true);
    }
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
      //ESP_LOGE("RcEcuBus", "I2C command failed to 0x%02X: %s", address, esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
  }

  /**
   * Send a command without data to the specified client
   * @param command - Command to send
   * @param address - address of the client
   */
  void sendCmd(uint8_t command, uint8_t address) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);

    // Regular I2C address (7-bit)
    i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, true);

    i2c_master_write_byte(cmd, command, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) {
      //ESP_LOGE("RcEcuBus", "I2C command failed to 0x%02X: %s", address, esp_err_to_name(ret));
    }
    i2c_cmd_link_delete(cmd);
  }

  /**
   * Request data from specific client
   * @param command - Command to request data
   * @param address - Address of the client
   * @returns Data received from the client, or 0xFFFF on error
   */
  uint16_t readData(uint8_t command, uint8_t address) {
    uint8_t buffer[2] = {0xFF, 0xFF};

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, command, true);
    i2c_master_start(cmd);  // Repeated start for read
    i2c_master_write_byte(cmd, address << 1 | I2C_MASTER_READ, true);
    i2c_master_read(cmd, buffer, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
      ESP_LOGE("RcEcuBus", "I2C read failed from address 0x%02X: %s", address, esp_err_to_name(ret));
      return 0xFFFF;
    }

    return buffer[0] << 8 | buffer[1];
  }

private:
  int _sda;
  int _scl;
  uint8_t _registeredBoards[MAX_BOARDS];
  uint8_t _boardCount = 0;
};