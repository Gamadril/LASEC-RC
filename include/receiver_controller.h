#pragma once

#include "board_config.h"

#include "SUMD.h"
#include "esp_timer.h"

// Receiver timeout constants
#define RECEIVER_TIMEOUT_MS 100   // 100ms timeout for no signal failsafe
#define RECEIVER_FRAME_RATE_MS 10 // Expected ~10ms frame rate (100Hz)

class ReceiverController {
public:
  typedef void (*on_frame_callback_t)(uint16_t *channels, uint8_t channels_count, bool failsafe);

  ReceiverController(uart_port_t uart_port) : _sumd(uart_port, PIN_RECEIVER) {
  }

  void init(on_frame_callback_t frame_callback) {
    _on_frame_callback = frame_callback;
    _last_frame_time = esp_timer_get_time() / 1000ULL; // Initialize to current time
    _timeout_failsafe_triggered = false;

    _sumd.init();
    _sumd.set_callback(&ReceiverController::on_sumd_frame, this);
  }

  // Check for receiver timeout and trigger failsafe if needed
  void checkTimeout() {
    uint32_t current_time = esp_timer_get_time() / 1000ULL;
    uint32_t time_since_last_frame = current_time - _last_frame_time;

    if (time_since_last_frame > RECEIVER_TIMEOUT_MS) {
      if (!_timeout_failsafe_triggered) {
        _timeout_failsafe_triggered = true;
        // Trigger failsafe with last known channels but failsafe=true
        _on_frame_callback(_channels, kChannelsCapacity, true);
        ESP_LOGW("RX", "Receiver timeout after %dms - triggering failsafe", time_since_last_frame);
      }
    }
  }

private:
  static constexpr uint8_t kChannelsCapacity = 16;
  uint16_t _channels[kChannelsCapacity];
  bool _failsafe;
  SUMD _sumd;
  on_frame_callback_t _on_frame_callback;

  // Timeout failsafe tracking
  uint32_t _last_frame_time;
  bool _timeout_failsafe_triggered;

  static void on_sumd_frame(const SUMD::SUMD_Frame *frame, void *user_ctx) {
    ReceiverController *rc = static_cast<ReceiverController *>(user_ctx);
    uint8_t n = std::min(frame->channels_count, kChannelsCapacity);
    for (uint8_t i = 0; i < n; i++)
      rc->_channels[i] = frame->channels[i];
    rc->_failsafe = frame->failsafe;

    // Update timeout tracking - we received a valid frame
    rc->_last_frame_time = esp_timer_get_time() / 1000ULL;
    rc->_timeout_failsafe_triggered = false; // Clear timeout failsafe

    rc->_on_frame_callback(rc->_channels, n, rc->_failsafe);
  }
};
