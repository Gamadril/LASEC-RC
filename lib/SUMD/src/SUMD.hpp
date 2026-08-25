/*
 * SUMD.hpp - Library for reading data from receivers that use SUMD protocol
 * MIT License
 */

#pragma once

#include <cstdint>
#if defined(ESP_PLATFORM)
#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#elif defined(__linux__)
#include <atomic>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <termios.h>
#include <thread>
#include <unistd.h>
#endif

#define SUMD_BAUD 115200

#define SUMD_VENDOR_ID 0xA8           // Graupner
#define SUMD_DATA_FRAME 0x01          // valid and live data frame
#define SUMD_DATA_FRAME_FAILSAFE 0x81 // valid data frame with transmitter in fail safe condition
#define CRC_POLYNOME 0x1021

#define CHANNEL_MIN_VALUE 1100
#define CHANNEL_MAX_VALUE 1900

// HoTT SUMD Protocol Constants per specification
#define SUMD_MIN_CHANNELS 2                                   // Minimum valid channel count
#define SUMD_MAX_CHANNELS 32                                  // Maximum valid channel count
#define SUMD_MAX_FRAME_LENGTH (3 + SUMD_MAX_CHANNELS * 2 + 2) // 3 header + N*2 + 2 CRC
#define SUMD_RAW_MIN_VALUE 0x1c20                             // Extended low position (-150%)
#define SUMD_RAW_MAX_VALUE 0x41a0                             // Extended high position (+150%)
#define SUMD_RAW_NEUTRAL 0x2ee0                               // Neutral position (0%)

// Frame timing validation (spec: 100Hz = 10ms intervals)
#define SUMD_FRAME_TIMEOUT_MS 15     // Allow 15ms for timing variations
#define SUMD_MIN_FRAME_INTERVAL_MS 8 // Minimum frame interval (faster than 125Hz)

#if defined(ESP_PLATFORM)
#define LOGE ESP_LOGE
#define LOGW ESP_LOGW
#elif defined(__linux__)
#define LOGE(TAG, fmt, ...) printf("[ERROR] %s: " fmt "\n", TAG, ##__VA_ARGS__)
#define LOGW(TAG, fmt, ...) printf("[WARN ] %s: " fmt "\n", TAG, ##__VA_ARGS__)
#endif

struct SUMD_Frame {
  uint16_t channels[SUMD_MAX_CHANNELS];
  uint8_t channels_count;
  bool failsafe;
};

class SUMD {
public:
  typedef void (*on_frame_callback_t)(const SUMD_Frame *frame, void *user_ctx);
#if defined(ESP_PLATFORM)
  SUMD(uart_port_t uart_port, int rx_pin) : _uart_port(uart_port), _rx_pin(rx_pin) {
  }
#elif defined(__linux__)
  SUMD(const char *dev_path) : _dev_path(dev_path) {
  }
#endif

  void init() {
#if defined(ESP_PLATFORM)
    uart_config_t uart_config = {};

    uart_config.baud_rate = SUMD_BAUD;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    ESP_ERROR_CHECK(uart_driver_install(_uart_port, 2048, 0, 20, &_uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(_uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(_uart_port, UART_PIN_NO_CHANGE, _rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    xTaskCreatePinnedToCore(&_reader_task_trampoline, "sumd_uart_reader", 3072, this, 10, &_reader_task,
                            tskNO_AFFINITY);
#elif defined(__linux__)
    if (_dev_path == nullptr) {
      LOGE(TAG, "no device path specified");
      return;
    }

    _fd = open(_dev_path, O_RDONLY | O_NOCTTY);
    if (_fd < 0) {
      LOGE(TAG, "open uart device %s failed", _dev_path);
      return;
    }

    struct termios tty{};
    if (tcgetattr(_fd, &tty) != 0) {
      LOGE(TAG, "tcgetattr failed");
      close(_fd);
      return;
    }

    cfmakeraw(&tty); // raw mode for binary data

    // Set baud rate
    cfsetispeed(&tty, SUMD_BAUD);
    cfsetospeed(&tty, SUMD_BAUD);

    // Blocking read: wait for 1 byte
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    tcflush(_fd, TCIFLUSH);

    if (tcsetattr(_fd, TCSANOW, &tty) != 0) {
      LOGE(TAG, "tcsetattr failed");
      close(_fd);
      return;
    }

    _running.store(true);
    _reader_thread = std::thread(_reader_task_trampoline, this);
#endif
  }

  void deinit() {
#if defined(ESP_PLATFORM)
    vTaskDelete(_reader_task);
    uart_driver_delete(_uart_port);
#elif defined(__linux__)
    _running.store(false);
    close(_fd);
    _reader_thread.join();
#endif
  }

  void set_callback(on_frame_callback_t cb, void *user_ctx) {
    _cb = cb;
    _cb_user = user_ctx;
  }

private:
  static inline const char *TAG = "SUMD";
  on_frame_callback_t _cb = NULL;
  void *_cb_user = NULL;
  bool _debug_enabled = false;

#if defined(ESP_PLATFORM)
  uart_port_t _uart_port;
  int _rx_pin;
  QueueHandle_t _uart_queue = NULL;
  TaskHandle_t _reader_task = NULL;
#elif defined(__linux__)
  int _fd = -1;
  const char *_dev_path;
  std::atomic<bool> _running;
  std::thread _reader_thread;
#endif

  int32_t _decode_frame_to_struct(uint8_t *frame, SUMD_Frame *out_frame, uint8_t channels) {
    // Validate frame status byte per specification
    if (frame[1] != SUMD_DATA_FRAME && frame[1] != SUMD_DATA_FRAME_FAILSAFE) {
      return 1;
    }

    out_frame->failsafe = frame[1] == SUMD_DATA_FRAME_FAILSAFE;
    uint16_t frame_size = 3 + channels * 2;

    uint16_t frame_crc = (frame[frame_size] << 8) | frame[frame_size + 1];

    // Compute CRC over header + data, per spec
    uint16_t crc = 0;
    for (uint8_t i = 0; i < frame_size; i++) {
      crc = this->CRC16(crc, frame[i]);
    }

    if (frame_crc != crc) {
      LOGE(TAG, "SUMD checksum error: 0x%04X != 0x%04X", frame_crc, crc);
      return 3;
    }

    // Validate and copy channel data
    out_frame->channels_count = channels;
    for (uint8_t i = 0; i < channels; i++) {
      uint16_t raw_value = (frame[3 + i * 2] << 8) | frame[4 + i * 2];

      // Validate channel data range per specification
      if (raw_value < SUMD_RAW_MIN_VALUE || raw_value > SUMD_RAW_MAX_VALUE) {
        if (_debug_enabled) {
          LOGW(TAG, "Channel %d raw value out of range: 0x%04X (must be 0x%04X-0x%04X)", i, raw_value,
               SUMD_RAW_MIN_VALUE, SUMD_RAW_MAX_VALUE);
        }
        return 5; // Channel data out of range
      }

      out_frame->channels[i] = raw_value / 8; // Convert to microseconds per spec
    }

    return 0;
  }

  static void _reader_task_trampoline(void *arg) {
    static_cast<SUMD *>(arg)->_reader_task_fn();
  }

  void _reader_task_fn() {
    uint8_t frame[SUMD_MAX_FRAME_LENGTH];
    uint8_t idx = 0;
    uint8_t expected_length = 0;
    uint8_t real_channels = 0;
#if defined(ESP_PLATFORM)
    uart_event_t event;
#endif

#if defined(ESP_PLATFORM)
    while (true) {
      vTaskDelay(pdMS_TO_TICKS(10));

      if (xQueueReceive(_uart_queue, &event, portMAX_DELAY) != pdTRUE) {
        continue;
      }

      switch (event.type) {
        case UART_DATA: {
          int remaining = event.size;
          while (remaining > 0) {
            uint8_t rb;
            int len = uart_read_bytes(_uart_port, &rb, 1, 0);
            if (len <= 0)
              break;
            remaining -= len;

            if (idx == 0 && rb != SUMD_VENDOR_ID) {
              continue;
            }
            if (idx == 1 && (rb != SUMD_DATA_FRAME && rb != SUMD_DATA_FRAME_FAILSAFE)) {
              idx = 0;
              expected_length = 0;
              continue;
            }
            if (idx >= SUMD_MAX_FRAME_LENGTH) {
              idx = 0;
              expected_length = 0;
              continue;
            }
            frame[idx++] = rb;
            if (idx == 3) {
              real_channels = frame[2];
              if (real_channels < SUMD_MIN_CHANNELS || real_channels > SUMD_MAX_CHANNELS) {
                if (_debug_enabled) {
                  LOGE(TAG, "Invalid channel count in frame: %d (must be %d-%d)", real_channels, SUMD_MIN_CHANNELS,
                       SUMD_MAX_CHANNELS);
                }
                idx = 0;
                expected_length = 0;
                continue;
              }
              expected_length = 3 + real_channels * 2 + 2;
              if (expected_length > SUMD_MAX_FRAME_LENGTH) {
                if (_debug_enabled) {
                  LOGW(TAG, "Frame too long: %d bytes (max %d)", expected_length, SUMD_MAX_FRAME_LENGTH);
                }
                idx = 0;
                expected_length = 0;
                continue;
              }
            }
            if (expected_length && idx == expected_length) {
              SUMD_Frame outf;
              if (_decode_frame_to_struct(frame, &outf, real_channels) == 0) {
                if (_cb) {
                  _cb(&outf, _cb_user);
                }
              }
              idx = 0;
              expected_length = 0;
            }
          }
          break;
        }
        case UART_FIFO_OVF:
        case UART_BUFFER_FULL:
          uart_flush_input(_uart_port);
          xQueueReset(_uart_queue);
          idx = 0;
          expected_length = 0;
          break;
        default:
          break;
      }
    }
#elif defined(__linux__)
    while (_running.load()) {
      uint8_t rb;
      int len = read(_fd, &rb, 1);

      if (len < 0) {
        // Equivalent to UART FIFO OVF or BUFFER FULL: reset parser
        if (_debug_enabled) {
          LOGE(TAG, "UART read error: %s", strerror(errno));
        }
        idx = 0;
        expected_length = 0;
        continue;
      }
      if (len == 0) {
        // Should not happen in blocking mode, but handle anyway
        continue;
      }

      if (idx == 0 && rb != SUMD_VENDOR_ID) {
        continue;
      }
      if (idx == 1 && (rb != SUMD_DATA_FRAME && rb != SUMD_DATA_FRAME_FAILSAFE)) {
        idx = 0;
        expected_length = 0;
        continue;
      }
      if (idx >= SUMD_MAX_FRAME_LENGTH) {
        idx = 0;
        expected_length = 0;
        continue;
      }

      frame[idx++] = rb;

      if (idx == 3) {
        real_channels = frame[2];

        if (real_channels < SUMD_MIN_CHANNELS || real_channels > SUMD_MAX_CHANNELS) {
          if (_debug_enabled) {
            LOGE(TAG, "Invalid channel count: %d", real_channels);
          }
          idx = 0;
          expected_length = 0;
          continue;
        }

        expected_length = 3 + real_channels * 2 + 2;

        if (expected_length > SUMD_MAX_FRAME_LENGTH) {
          if (_debug_enabled) {
            LOGE(TAG, "Frame too long: %d", expected_length);
          }
          idx = 0;
          expected_length = 0;
          continue;
        }
      }

      if (expected_length && idx == expected_length) {
        SUMD_Frame out_frame;

        if (_decode_frame_to_struct(frame, &out_frame, real_channels) == 0) {
          if (_cb)
            _cb(&out_frame, _cb_user);
        }

        idx = 0;
        expected_length = 0;
      }
    }
#endif
  }

  uint16_t CRC16(uint16_t crc, uint8_t value) {
    crc = crc ^ ((uint16_t)value << 8);
    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ CRC_POLYNOME;
      else
        crc = (crc << 1);
    }
    return crc;
  }
};
