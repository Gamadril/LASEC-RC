/*
 * SUMD.h - Library for reading data from receivers that use SUMD protocol
 * MIT License
 */

#pragma once

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <algorithm> // for std::min/max

#define SUMD_BAUD 115200
#define SUMD_MAX_ONWIRE_CHANNELS 32                                  // per spec
#define SUMD_MAX_FRAME_LENGTH (3 + SUMD_MAX_ONWIRE_CHANNELS * 2 + 2) // 3 header + N*2 + 2 CRC

#define SUMD_VENDOR_ID 0xA8           // Graupner
#define SUMD_DATA_FRAME 0x01          // valid and live data frame
#define SUMD_DATA_FRAME_FAILSAFE 0x81 // valid data frame with transmitter in fail safe condition
#define CRC_POLYNOME 0x1021

#define CHANNEL_MIN_VALUE 1100
#define CHANNEL_MAX_VALUE 1900

// HoTT SUMD Protocol Constants per specification
#define SUMD_MIN_CHANNELS 2          // Minimum valid channel count
#define SUMD_MAX_CHANNELS 32         // Maximum valid channel count
#define SUMD_RAW_MIN_VALUE 0x1c20    // Extended low position (-150%)
#define SUMD_RAW_MAX_VALUE 0x41a0    // Extended high position (+150%)
#define SUMD_RAW_NEUTRAL 0x2ee0      // Neutral position (0%)

// Frame timing validation (spec: 100Hz = 10ms intervals)
#define SUMD_FRAME_TIMEOUT_MS 15     // Allow 15ms for timing variations
#define SUMD_MIN_FRAME_INTERVAL_MS 8 // Minimum frame interval (faster than 125Hz)

class SUMD {
public:
  struct SUMD_Frame {
    uint16_t channels[SUMD_MAX_ONWIRE_CHANNELS];
    uint8_t channels_count;
    bool failsafe;
    uint32_t frame_number;        // Sequential frame counter
    uint32_t timestamp_ms;        // When frame was received
    uint16_t raw_crc;             // Raw CRC from frame for debugging
    bool timing_valid;            // Frame timing is within spec
  };

  struct SUMD_Stats {
    uint32_t total_frames;        // Total frames received
    uint32_t valid_frames;        // Frames with valid CRC
    uint32_t invalid_frames;      // Frames with invalid CRC
    uint32_t failsafe_frames;     // Frames in failsafe mode
    uint32_t timing_errors;       // Frames with invalid timing
    uint32_t last_frame_ms;       // Timestamp of last valid frame
    uint16_t avg_frame_interval;  // Average interval between frames (ms)
    uint8_t min_channels;         // Minimum channels seen
    uint8_t max_channels;         // Maximum channels seen
  };

  typedef void (*on_frame_callback_t)(const SUMD_Frame *frame, void *user_ctx);
  SUMD(uart_port_t uart_port, int rx_pin) : _uart_port(uart_port), _rx_pin(rx_pin) {
    // UART buffer size is configured during driver installation
  }

  void init() {
    uart_config_t uart_config = {};

    uart_config.baud_rate = SUMD_BAUD;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    ESP_ERROR_CHECK(uart_driver_install(_uart_port, 2048, 0, 20, &_uart_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(_uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(_uart_port, UART_PIN_NO_CHANGE, _rx_pin, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    if (_reader_task == NULL) {
      xTaskCreatePinnedToCore(&SUMD::_reader_task_trampoline, "sumd_uart_reader", 3072, this, 10,
                              &_reader_task, tskNO_AFFINITY);
    }
  }

  void set_callback(on_frame_callback_t cb, void *user_ctx) {
    _cb = cb;
    _cb_user = user_ctx;
  }

  // Get frame statistics
  void get_stats(SUMD_Stats *stats) {
    *stats = _stats;
  }

  // Reset frame statistics
  void reset_stats() {
    memset(&_stats, 0, sizeof(_stats));
    _stats.min_channels = 255; // Initialize to max value
    _stats.max_channels = 0;
    _last_frame_time = 0;
  }

  // Enable/disable debug output
  void set_debug(bool debug) {
    _debug_enabled = debug;
  }

  // Old polling-based read(...) methods removed in favor of callback-based reception

private:
  uart_port_t _uart_port;
  int _rx_pin;
  QueueHandle_t _uart_queue = NULL;
  TaskHandle_t _reader_task = NULL;
  on_frame_callback_t _cb = NULL;
  void *_cb_user = NULL;
  bool _debug_enabled = false;

  // Statistics tracking
  SUMD_Stats _stats = {};
  uint32_t _last_frame_time = 0;
  uint32_t _frame_counter = 0;

  // no internal channel buffer; caller provides capacity or uses SUMD_Frame API

  int32_t _decode_frame(uint8_t *frame, uint16_t *channels, uint8_t channels_capacity,
                        uint8_t *out_channels_count, bool *failsafe, uint8_t onwire_channels) {
    // Validate frame status byte per specification
    if (frame[1] != SUMD_DATA_FRAME && frame[1] != SUMD_DATA_FRAME_FAILSAFE) {
      // Values different to 0x01 or 0x81 indicate an invalid SUMD data frame and should not be
      // processed by SUMD algorithms.
      return 1;
    }

    // Validate channel count per specification (2-32 channels)
    if (onwire_channels < SUMD_MIN_CHANNELS || onwire_channels > SUMD_MAX_CHANNELS) {
      if (_debug_enabled) {
        ESP_LOGW("SUMD", "Invalid channel count: %d (must be %d-%d)", onwire_channels,
                SUMD_MIN_CHANNELS, SUMD_MAX_CHANNELS);
      }
      return 4; // Invalid channel count
    }

    *failsafe = frame[1] == SUMD_DATA_FRAME_FAILSAFE;

    // Compute CRC over header + data, per spec
    const uint16_t frame_crc =
        (frame[3 + onwire_channels * 2] << 8) | frame[3 + onwire_channels * 2 + 1];
    uint16_t crc = 0;
    for (uint8_t i = 0; i < 3 + onwire_channels * 2; i++) {
      crc = this->CRC16(crc, frame[i]);
    }

    if (frame_crc != crc) {
      // Reduce error logging frequency to avoid flooding
      static unsigned long last_error_time = 0;
      if ((esp_timer_get_time() / 1000ULL) - last_error_time > 1000) { // Log max once per second
        ESP_LOGE("SUMD", "SUMD checksum error: 0x%04X != 0x%04X", frame_crc, crc);
        last_error_time = (esp_timer_get_time() / 1000ULL);
      }
      return 3;
    }

    // Validate and copy channel data
    const uint8_t copy_channels =
        onwire_channels > channels_capacity ? channels_capacity : onwire_channels;

    for (uint8_t i = 0; i < copy_channels; i++) {
      uint16_t raw_value = (frame[3 + i * 2] << 8) | frame[4 + i * 2];

      // Validate channel data range per specification
      if (raw_value < SUMD_RAW_MIN_VALUE || raw_value > SUMD_RAW_MAX_VALUE) {
        if (_debug_enabled) {
          ESP_LOGW("SUMD", "Channel %d raw value out of range: 0x%04X (must be 0x%04X-0x%04X)",
                  i, raw_value, SUMD_RAW_MIN_VALUE, SUMD_RAW_MAX_VALUE);
        }
        return 5; // Channel data out of range
      }

      channels[i] = raw_value / 8; // Convert to microseconds per spec

      if (_debug_enabled && i < 4) { // Debug first 4 channels
        ESP_LOGV("SUMD", "Ch%d: raw=0x%04X, scaled=%dµs", i, raw_value, channels[i]);
      }
    }

    if (out_channels_count) {
      *out_channels_count = copy_channels;
    }
    return 0;
  }

  int32_t _decode_frame_to_struct(uint8_t *frame, SUMD_Frame *out_frame, uint8_t onwire_channels) {
    // Validate frame status byte per specification
    if (frame[1] != SUMD_DATA_FRAME && frame[1] != SUMD_DATA_FRAME_FAILSAFE) {
      return 1;
    }

    // Validate channel count per specification (2-32 channels)
    if (onwire_channels < SUMD_MIN_CHANNELS || onwire_channels > SUMD_MAX_CHANNELS) {
      if (_debug_enabled) {
        ESP_LOGW("SUMD", "Invalid channel count: %d (must be %d-%d)", onwire_channels,
                SUMD_MIN_CHANNELS, SUMD_MAX_CHANNELS);
      }
      return 4; // Invalid channel count
    }

    out_frame->failsafe = frame[1] == SUMD_DATA_FRAME_FAILSAFE;

    // Store raw CRC for debugging
    out_frame->raw_crc = (frame[3 + onwire_channels * 2] << 8) | frame[3 + onwire_channels * 2 + 1];

    // Compute CRC over header + data, per spec
    uint16_t crc = 0;
    for (uint8_t i = 0; i < 3 + onwire_channels * 2; i++) {
      crc = this->CRC16(crc, frame[i]);
    }

    if (out_frame->raw_crc != crc) {
      _stats.invalid_frames++;
      static unsigned long last_error_time = 0;
      if ((esp_timer_get_time() / 1000ULL) - last_error_time > 1000) {
        ESP_LOGE("SUMD", "SUMD checksum error: 0x%04X != 0x%04X", out_frame->raw_crc, crc);
        last_error_time = (esp_timer_get_time() / 1000ULL);
      }
      return 3;
    }

    // Validate frame timing (should be ~10ms intervals per spec)
    uint32_t current_time = esp_timer_get_time() / 1000ULL;
    out_frame->timing_valid = true;
    if (_last_frame_time > 0) {
      uint32_t interval = current_time - _last_frame_time;
      if (interval < SUMD_MIN_FRAME_INTERVAL_MS || interval > SUMD_FRAME_TIMEOUT_MS) {
        out_frame->timing_valid = false;
        _stats.timing_errors++;
        if (_debug_enabled) {
          ESP_LOGW("SUMD", "Frame timing error: %dms (expected ~10ms)", interval);
        }
      }
    }
    _last_frame_time = current_time;

    // Update statistics
    _stats.total_frames++;
    _stats.valid_frames++;
    if (out_frame->failsafe) {
      _stats.failsafe_frames++;
    }
    _stats.min_channels = std::min(_stats.min_channels, onwire_channels);
    _stats.max_channels = std::max(_stats.max_channels, onwire_channels);

    // Calculate average frame interval
    if (_stats.valid_frames > 1) {
      uint32_t total_interval = current_time - (_stats.last_frame_ms - _stats.avg_frame_interval);
      _stats.avg_frame_interval = total_interval / (_stats.valid_frames - 1);
    }
    _stats.last_frame_ms = current_time;

    // Set frame metadata
    out_frame->frame_number = ++_frame_counter;
    out_frame->timestamp_ms = current_time;

    // Validate and copy channel data
    out_frame->channels_count = onwire_channels;
    for (uint8_t i = 0; i < onwire_channels; i++) {
      uint16_t raw_value = (frame[3 + i * 2] << 8) | frame[4 + i * 2];

      // Validate channel data range per specification
      if (raw_value < SUMD_RAW_MIN_VALUE || raw_value > SUMD_RAW_MAX_VALUE) {
        if (_debug_enabled) {
          ESP_LOGW("SUMD", "Channel %d raw value out of range: 0x%04X (must be 0x%04X-0x%04X)",
                  i, raw_value, SUMD_RAW_MIN_VALUE, SUMD_RAW_MAX_VALUE);
        }
        return 5; // Channel data out of range
      }

      out_frame->channels[i] = raw_value / 8; // Convert to microseconds per spec

      if (_debug_enabled && i < 4) { // Debug first 4 channels
        ESP_LOGV("SUMD", "Ch%d: raw=0x%04X, scaled=%dµs", i, raw_value, out_frame->channels[i]);
      }
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
    uint8_t onwire_channels = 0;

    for (;;) {
      uart_event_t event;
      if (_uart_queue == NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
        continue;
      }
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
            onwire_channels = frame[2];
            if (onwire_channels < SUMD_MIN_CHANNELS || onwire_channels > SUMD_MAX_CHANNELS) {
              if (_debug_enabled) {
                ESP_LOGW("SUMD", "Invalid channel count in frame: %d (must be %d-%d)",
                        onwire_channels, SUMD_MIN_CHANNELS, SUMD_MAX_CHANNELS);
              }
              idx = 0;
              expected_length = 0;
              continue;
            }
            expected_length = 3 + onwire_channels * 2 + 2;
            if (expected_length > SUMD_MAX_FRAME_LENGTH) {
              if (_debug_enabled) {
                ESP_LOGW("SUMD", "Frame too long: %d bytes (max %d)",
                        expected_length, SUMD_MAX_FRAME_LENGTH);
              }
              idx = 0;
              expected_length = 0;
              continue;
            }
          }
          if (expected_length && idx == expected_length) {
            SUMD_Frame outf;
            if (_decode_frame_to_struct(frame, &outf, onwire_channels) == 0) {
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
        if (_uart_queue)
          xQueueReset(_uart_queue);
        idx = 0;
        expected_length = 0;
        break;
      default:
        break;
      }
    }
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
