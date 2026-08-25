#pragma once

#include "SUMD.hpp"
#include "signal.hpp"
#include "utils.hpp"
#include <cstdint>

// Receiver timeout constants
#define RECEIVER_TIMEOUT_MS 100   // 100ms timeout for no signal failsafe
#define RECEIVER_FRAME_RATE_MS 10 // Expected ~10ms frame rate (100Hz)

class ReceiverController {
public:
  sigslot::signal<uint16_t *, uint8_t, bool> onFrame;

#if defined(ESP_PLATFORM)
  ReceiverController(uart_port_t uart_port, int rx_pin) : _sumd(uart_port, rx_pin) {
  }
#elif defined(__linux__)
  ReceiverController(const char *dev_path) : _sumd(dev_path) {
  }
#endif

  void init() {
#if defined(ESP_PLATFORM)
    _last_frame_time = millis(); // Initialize to current time
#elif defined(__linux__)
    _last_frame_time =
        std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch())
            .count();
#endif
    _timeout_failsafe_triggered = false;

    _sumd.init();
    _sumd.set_callback(&ReceiverController::on_sumd_frame, this);
  }

  // Check for receiver timeout and trigger failsafe if needed
  void checkTimeout() {
    int64_t current_time = millis();
    int64_t time_since_last_frame = current_time - _last_frame_time;

    if (time_since_last_frame > RECEIVER_TIMEOUT_MS) {
      if (!_timeout_failsafe_triggered) {
        _timeout_failsafe_triggered = true;
        // Trigger failsafe with last known channels but failsafe=true
        onFrame(_channels, _nr_channels, true);
        ESP_LOGW(TAG, "Receiver timeout after %ldms - triggering failsafe", time_since_last_frame);
      }
    }
  }

private:
  static inline const char *TAG = "RCV";
  uint16_t _channels[SUMD_MAX_CHANNELS];
  uint16_t _last_channels[SUMD_MAX_CHANNELS];
  uint8_t _nr_channels;
  bool _failsafe;
  bool _last_failsafe;
  SUMD _sumd;

  // Timeout failsafe tracking
  int64_t _last_frame_time;
  bool _timeout_failsafe_triggered;

  static void on_sumd_frame(const SUMD_Frame *frame, void *user_ctx) {
    ReceiverController *rc = static_cast<ReceiverController *>(user_ctx);

    rc->_nr_channels = frame->channels_count;
    rc->_failsafe = frame->failsafe;

    // Copy only valid channels
    size_t data_size = rc->_nr_channels * sizeof(uint16_t);
    memcpy(rc->_channels, frame->channels, data_size);

    // Update timeout tracking - we received a valid frame
    rc->_last_frame_time = millis();
    rc->_timeout_failsafe_triggered = false; // Clear timeout failsafe

    // Compare only the valid part of the arrays

    bool changed = (memcmp(rc->_last_channels, rc->_channels, data_size) != 0) || (rc->_failsafe != rc->_last_failsafe);

    if (changed) {
      memcpy(rc->_last_channels, rc->_channels, data_size);
      rc->_last_failsafe = rc->_failsafe;

      rc->onFrame(rc->_channels, rc->_nr_channels, rc->_failsafe);
    }
  }
};
