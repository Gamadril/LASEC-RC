#pragma once

#include "driver/gptimer.h"
#include "esp_err.h"
#include "esp_log.h"
#include "signal.hpp"

#include "../hal/timer.hpp"

class TimerESP : public Timer {
public:
  sigslot::signal<> onAlarm;

  TimerESP(uint32_t period_ms) : _timer_handle(nullptr), _period_ms(period_ms) {
    esp_err_t err;

    gptimer_config_t timer_config = {};
    timer_config.clk_src = GPTIMER_CLK_SRC_DEFAULT; // Select the default clock source
    timer_config.direction = GPTIMER_COUNT_UP;      // Counting direction is up
    timer_config.resolution_hz = 1000000;           // 1 MHz resolution for 1 us tick

    // Create a timer instance
    err = gptimer_new_timer(&timer_config, &_timer_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "gptimer fixed new failed: %d", err);
      return;
    }

    // Configure alarm
    gptimer_alarm_config_t alarm_config = {};
    alarm_config.alarm_count = _period_ms * 1000;   // Convert ms to microseconds
    alarm_config.reload_count = 0;                  // start from 0
    alarm_config.flags.auto_reload_on_alarm = true; // periodic mode
    err = gptimer_set_alarm_action(_timer_handle, &alarm_config);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "gptimer fixed set_alarm failed: %d", err);
      return;
    }

    // Attach ISR
    gptimer_event_callbacks_t cbs = {
        .on_alarm = &_timer_static_cb,
    };
    err = gptimer_register_event_callbacks(_timer_handle, &cbs, this);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "gptimer fixed cb reg failed: %d", err);
      return;
    }

    // Enable timer
    err = gptimer_enable(_timer_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "gptimer fixed enable failed: %d", err);
      return;
    }

    ESP_LOGI(TAG, "Timer initialized with period %ums", _period_ms);
  }

  ~TimerESP() {
    stop();
    if (_timer_handle) {
      gptimer_disable(_timer_handle);
      gptimer_del_timer(_timer_handle);
    }
  }

  void start() override {
    ESP_LOGI(TAG, "Starting timer...");
    esp_err_t err = gptimer_start(_timer_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "gptimer start failed: %d", err);
      return;
    }
    _timerRunning = true;
  }

  void stop() override {
    ESP_LOGI(TAG, "Stopping timer...");
    esp_err_t err = gptimer_stop(_timer_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "gptimer stop failed: %d", err);
      return;
    }
    _timerRunning = false;
  }

  bool is_running() override {
    return _timerRunning;
  }

private:
  gptimer_handle_t _timer_handle;
  uint32_t _period_ms;
  bool _timerRunning = false;

  /**
   * @brief The static C-style callback required by ESP-IDF API (Trampoline).
   */
  static bool _timer_static_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx) {
    // Cast the user_ctx back to a pointer to our C++ class instance
    TimerESP *instance = static_cast<TimerESP *>(user_ctx);

    if (instance) {
      // EMIT the sigslot signal from within the ISR context
      instance->onAlarm();
    }

    // Return false to indicate no high-priority task needs switching
    return false;
  }
};
