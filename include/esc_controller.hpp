#pragma once

#include "board_config.hpp"
#include "common.hpp"
#include "utils.hpp"
#if defined(ESP_PLATFORM)
#include "driver/mcpwm_prelude.h"
#include "esp_log.h"
#endif

#define ESC_DEFAULT_FREQUENCY 50

// 1 tick = 1 microsecond, so compare values map 1:1 to microseconds
#define ESC_TIMEBASE_RESOLUTION_HZ 1000000

class EscController {
public:
  EscController() = default;

  bool init(Config *config) {
    _config = config;

    if (_config == nullptr) {
      ESP_LOGE(TAG, "ESC config is null");
      return false;
    }

#if defined(ESP_PLATFORM)
    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = 1;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = ESC_TIMEBASE_RESOLUTION_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = ESC_TIMEBASE_RESOLUTION_HZ / _frequency;

    esp_err_t err = mcpwm_new_timer(&timer_config, &_timer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_new_timer failed: %s", esp_err_to_name(err));
      return false;
    }

    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = 1;

    err = mcpwm_new_operator(&operator_config, &_oper);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_new_operator failed: %s", esp_err_to_name(err));
      return false;
    }

    err = mcpwm_operator_connect_timer(_oper, _timer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_operator_connect_timer failed: %s", esp_err_to_name(err));
      return false;
    }

    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;

    err = mcpwm_new_comparator(_oper, &comparator_config, &_comparator);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_new_comparator failed: %s", esp_err_to_name(err));
      return false;
    }

    mcpwm_generator_config_t generator_config = {};
    generator_config.gen_gpio_num = PIN_ESC;

    err = mcpwm_new_generator(_oper, &generator_config, &_generator);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_new_generator failed: %s", esp_err_to_name(err));
      return false;
    }

    err = mcpwm_comparator_set_compare_value(_comparator, _config->escConfig.neutral);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_comparator_set_compare_value failed: %s", esp_err_to_name(err));
      return false;
    }

    // Go high at the start of the timer period...
    err = mcpwm_generator_set_action_on_timer_event(
        _generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_generator_set_action_on_timer_event failed: %s", esp_err_to_name(err));
      return false;
    }

    // ...and go low once the compare value is reached
    err = mcpwm_generator_set_action_on_compare_event(
        _generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, _comparator, MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_generator_set_action_on_compare_event failed: %s", esp_err_to_name(err));
      return false;
    }

    err = mcpwm_timer_enable(_timer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_timer_enable failed: %s", esp_err_to_name(err));
      return false;
    }

    err = mcpwm_timer_start_stop(_timer, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_timer_start_stop failed: %s", esp_err_to_name(err));
      return false;
    }
#endif

    // Set initial neutral pulse
    set_esc_us(_config->escConfig.neutral);

    ESP_LOGI(TAG, "ESC controller initialized successfully (freq=%u Hz, range=%u-%u µs)", _frequency,
             _config->escConfig.min, _config->escConfig.max);
    return true;
  }

  /**
   * Set ESC throttle/brake position in microseconds
   * @param value Input value (typically 1000-2000µs)
   */
  void set_esc_us(uint16_t value) {
    if (value == _last_us)
      return;

    // Clamp to configured range to protect ESC
    if (value < _config->escConfig.min) {
      ESP_LOGW(TAG, "Value %d ot of range: range=%u-%u", value, _config->escConfig.min, _config->escConfig.max);
      value = _config->escConfig.min;
    } else if (value > _config->escConfig.max) {
      ESP_LOGW(TAG, "Value %d ot of range: range=%u-%u", value, _config->escConfig.min, _config->escConfig.max);
      value = _config->escConfig.max;
    }

#if defined(ESP_PLATFORM)
    // Update the comparator's compare value (in timer ticks == microseconds here)
    esp_err_t err = mcpwm_comparator_set_compare_value(_comparator, value);
    if (err == ESP_OK) {
      _last_us = value;
      ESP_LOGV(TAG, "ESC set to %u µs", value);
    } else {
      ESP_LOGE(TAG, "Failed to set ESC pulse: %s", esp_err_to_name(err));
    }
#endif
  }

  /**
   * Get current ESC position in microseconds
   * @return Current position
   */
  uint16_t get_esc_us() const { return _last_us; }

  /**
   * Set braking state
   * @param value true if braking
   */
  void setBraking(bool value) { _isBraking = value; }

  /**
   * Check if ESC is in braking mode
   * @return true if braking
   */
  bool isBraking() const { return _isBraking; }

  /**
   * Set driving state
   * @param value true if driving forward
   */
  void setDriving(bool value) { _isDriving = value; }

  /**
   * Check if ESC is in driving mode
   * @return true if driving forward
   */
  bool isDriving() const { return _isDriving; }

  /**
   * Set reverse state
   * @param value true if in reverse
   */
  void setInReverse(bool value) { _isInReverse = value; }

  /**
   * Check if ESC is in reverse mode
   * @return true if in reverse
   */
  bool isInReverse() const { return _isInReverse; }

private:
  static inline const char *TAG = "ESC";
  Config *_config = nullptr;
  uint16_t _last_us = 0;
  bool _isBraking = false;
  bool _isDriving = false;
  bool _isInReverse = false;
  uint32_t _frequency = ESC_DEFAULT_FREQUENCY;

#if defined(ESP_PLATFORM)
  mcpwm_timer_handle_t _timer = nullptr;
  mcpwm_oper_handle_t _oper = nullptr;
  mcpwm_cmpr_handle_t _comparator = nullptr;
  mcpwm_gen_handle_t _generator = nullptr;
#endif
};