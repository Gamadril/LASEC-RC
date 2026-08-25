#pragma once

#include <cstdint>
#include <cstring>
#if defined(ESP_PLATFORM)
#include "driver/i2c_master.h"
#include "esp_log.h"
#elif defined(__linux__)
#include <cstdio>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

#include "RcEcuBusCommands.hpp"

#define I2C_MASTER_FREQ_HZ 100000
#define MAX_BOARDS 8
#define I2C_TIMEOUT_MS 1000

class RcEcuBus {
public:
  /**
   * Default constructor
   */
  RcEcuBus(int sda_pin, int scl_pin) : _sda(sda_pin), _scl(scl_pin) {
  }
#if defined(__linux__)
  RcEcuBus(const char *dev_path) : _dev_path(dev_path) {
  }
#endif

  ~RcEcuBus() {
#if defined(ESP_PLATFORM)
    for (int i = 0; i < _boardCount; i++) {
      if (_deviceHandles[i] != nullptr) {
        i2c_master_bus_rm_device(_deviceHandles[i]);
      }
    }
    if (_busHandle != nullptr) {
      i2c_del_master_bus(_busHandle);
    }
#elif defined(__linux__)
    if (_fd >= 0) {
      close(_fd);
    }
#endif
  }

  /**
   * Init communication
   */
  void init() {
#if defined(ESP_PLATFORM)
    i2c_master_bus_config_t bus_config = {};
    bus_config.i2c_port = I2C_NUM_0;
    bus_config.sda_io_num = static_cast<gpio_num_t>(_sda);
    bus_config.scl_io_num = static_cast<gpio_num_t>(_scl);
    bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    bus_config.glitch_ignore_cnt = 7;
    bus_config.flags.enable_internal_pullup = false;

    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &_busHandle));
#elif defined(__linux__)
    _fd = open(_dev_path, O_RDWR);
    if (_fd < 0) {
      printf("%s: open i2c device %s failed\n", TAG, _dev_path);
    }
#endif
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
#if defined(ESP_PLATFORM)
      i2c_master_dev_handle_t dev_handle = nullptr;
      if (!addDevice(address, &dev_handle)) {
        return;
      }
      _deviceHandles[_boardCount] = dev_handle;
#endif
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
   * Send a command with data to all registered clients (alternative name)
   * @param command - Command to send
   * @param data - Data to send
   * @param datalen - Size of the data to send
   */
  void sendToAll(uint8_t command, uint8_t *data, uint8_t datalen) {
    for (int i = 0; i < _boardCount; i++) {
      sendCmd(command, data, datalen, _registeredBoards[i]);
    }
  }

  /**
   * Send a command without data to all registered clients (alternative name)
   * @param command - Command to send
   */
  void sendToAll(uint8_t command) {
    for (int i = 0; i < _boardCount; i++) {
      sendCmd(command, _registeredBoards[i]);
    }
  }

  /**
   * Get the number of registered boards
   * @return Number of registered boards
   */
  uint8_t getRegisteredBoardCount() {
    return _boardCount;
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
#if defined(ESP_PLATFORM)
    i2c_master_dev_handle_t dev = getDeviceHandle(address);
    bool isTemporary = false;
    if (dev == nullptr) {
      if (!addDevice(address, &dev)) {
        return;
      }
      isTemporary = true;
    }

    // Combine command + data into a single write buffer, since the new
    // driver issues one transaction per call (no manual command-link building)
    uint8_t buffer[256];
    buffer[0] = command;
    if (data != nullptr && datalen > 0) {
      memcpy(&buffer[1], data, datalen);
    }

    esp_err_t ret = i2c_master_transmit(dev, buffer, 1 + datalen, I2C_TIMEOUT_MS);
    if (ret != ESP_OK) {
      // ESP_LOGE(TAG, "I2C command failed to 0x%02X: %s", address, esp_err_to_name(ret));
    }

    if (isTemporary) {
      i2c_master_bus_rm_device(dev);
    }
#elif defined(__linux__)
    if (_fd < 0) {
      printf("%s: Bus not open\n", TAG);
      return;
    }

    // Set slave address for this transaction
    if (ioctl(_fd, I2C_SLAVE, address) < 0) {
      printf("%s: ioctl I2C_SLAVE failed\n", TAG);
      close(_fd);
      _fd = -1;
      return;
    }

    if (write(_fd, &command, 1) != 1) {
      printf("%s: write failed\n", TAG);
    }

    if (data != nullptr && datalen > 0 && write(_fd, data, datalen) != datalen) {
      printf("%s: write failed\n", TAG);
    }
#endif
  }

  /**
   * Send a command without data to the specified client
   * @param command - Command to send
   * @param address - address of the client
   */
  void sendCmd(uint8_t command, uint8_t address) {
    sendCmd(command, nullptr, 0, address);
  }

  /**
   * Request data from specific client
   * @param command - Command to request data
   * @param address - Address of the client
   * @returns Data received from the client, or 0xFFFF on error
   */
  uint16_t readData(uint8_t command, uint8_t address) {
    uint8_t buffer[2] = {0xFF, 0xFF};

#if defined(ESP_PLATFORM)
    i2c_master_dev_handle_t dev = getDeviceHandle(address);
    bool isTemporary = false;
    if (dev == nullptr) {
      if (!addDevice(address, &dev)) {
        return 0xFFFF;
      }
      isTemporary = true;
    }

    // Write the command, then repeated-start into a 2-byte read, in one transaction
    esp_err_t ret = i2c_master_transmit_receive(dev, &command, 1, buffer, sizeof(buffer), I2C_TIMEOUT_MS);

    if (isTemporary) {
      i2c_master_bus_rm_device(dev);
    }

    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "I2C read failed from address 0x%02X: %s", address, esp_err_to_name(ret));
      return 0xFFFF;
    }
#elif defined(__linux__)
    if (_fd < 0) {
      printf("%s: Bus not open\n", TAG);
      return 0xFFFF;
    }

    // Set slave address for this transaction
    if (ioctl(_fd, I2C_SLAVE, address) < 0) {
      printf("%s: ioctl I2C_SLAVE failed\n", TAG);
      close(_fd);
      _fd = -1;
      return 0xFFFF;
    }

    if (write(_fd, &command, 1) != 1) {
      printf("%s: write failed\n", TAG);
      return 0xFFFF;
    }

    if (read(_fd, buffer, sizeof(buffer)) != sizeof(buffer)) {
      printf("%s: read failed\n", TAG);
      return 0xFFFF;
    }
#endif

    return buffer[0] << 8 | buffer[1];
  }

private:
#if defined(ESP_PLATFORM)
  /**
   * Look up the cached device handle for a registered board address
   * @return Handle if the address is registered, nullptr otherwise
   */
  i2c_master_dev_handle_t getDeviceHandle(uint8_t address) {
    for (int i = 0; i < _boardCount; i++) {
      if (_registeredBoards[i] == address) {
        return _deviceHandles[i];
      }
    }
    return nullptr;
  }

  /**
   * Add a new I2C device to the bus for the given address
   * @param address - I2C address of the device
   * @param outHandle - Out: created device handle
   * @return true on success, false otherwise
   */
  bool addDevice(uint8_t address, i2c_master_dev_handle_t *outHandle) {
    i2c_device_config_t dev_cfg = {};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address = address;
    dev_cfg.scl_speed_hz = I2C_MASTER_FREQ_HZ;

    esp_err_t err = i2c_master_bus_add_device(_busHandle, &dev_cfg, outHandle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to add I2C device 0x%02X: %s", address, esp_err_to_name(err));
      return false;
    }
    return true;
  }
#endif

  static inline const char *TAG = "REB";
  int _sda;
  int _scl;
  uint8_t _registeredBoards[MAX_BOARDS];
  uint8_t _boardCount = 0;
#if defined(ESP_PLATFORM)
  i2c_master_bus_handle_t _busHandle = nullptr;
  i2c_master_dev_handle_t _deviceHandles[MAX_BOARDS] = {};
#elif defined(__linux__)
  int _fd = -1;
  const char *_dev_path;
#endif
};