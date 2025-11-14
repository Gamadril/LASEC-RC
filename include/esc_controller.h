#pragma once

#include "board_config.h"
#include "driver/mcpwm.h"
#include "esp_log.h"

// ESC timing constants
#define ESC_DEFAULT_MIN_US 1000
#define ESC_DEFAULT_MAX_US 2000
#define ESC_DEFAULT_NEUTRAL_US 1500
#define ESC_DEFAULT_FREQUENCY 50

// Configuration info structure
struct EscConfigInfo {
  uint16_t min_us;
  uint16_t max_us;
  uint16_t neutral_us;
  uint16_t frequency;
};

class EscController {
public:
  EscController() = default;

  /**
   * Initialize ESC controller with optional configuration
   * @param config Pointer to ESC configuration (optional, uses defaults if null)
   * @return true if initialization successful, false otherwise
   */
  bool init(const EscConfig* config = nullptr) {
    // Use provided config or defaults
    if (config) {
      _min_us = config->min;
      _max_us = config->max;
      _neutral_us = config->neutral;
      // Note: EscConfig doesn't have frequency, use default
      _frequency = ESC_DEFAULT_FREQUENCY;
    } else {
      _min_us = ESC_DEFAULT_MIN_US;
      _max_us = ESC_DEFAULT_MAX_US;
      _neutral_us = ESC_DEFAULT_NEUTRAL_US;
      _frequency = ESC_DEFAULT_FREQUENCY;
    }

    // Validate configuration
    if (!validateEscConfig()) {
      ESP_LOGE("EscController", "Invalid ESC configuration");
      return false;
    }

    esp_err_t err;

    // Initialize ESC GPIO
    err = mcpwm_gpio_init(MCPWM_UNIT_1, MCPWM0A, PIN_ESC);
    if (err != ESP_OK) {
      ESP_LOGE("EscController", "Failed to initialize ESC GPIO: %s", esp_err_to_name(err));
      return false;
    }

    // Configure MCPWM for ESC
    mcpwm_config_t pwm_config = {
        .frequency = static_cast<uint32_t>(_frequency),
        .cmpr_a = 0,        // duty cycle of PWMxA = 0
        .cmpr_b = 0,        // duty cycle of PWMxB = 0
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };

    err = mcpwm_init(MCPWM_UNIT_1, MCPWM_TIMER_0, &pwm_config);
    if (err != ESP_OK) {
      ESP_LOGE("EscController", "Failed to initialize ESC MCPWM: %s", esp_err_to_name(err));
      return false;
    }

    // Set initial neutral pulse
    set_esc_us(_neutral_us);

    ESP_LOGI("EscController", "ESC controller initialized successfully (freq=%u Hz, range=%u-%u µs)",
             _frequency, _min_us, _max_us);
    return true;
  }

  /**
   * Set ESC throttle/brake position in microseconds
   * @param value Input value (typically 1000-2000µs)
   */
  void set_esc_us(uint16_t value) {
    if (value == _last_us) return;

    // Clamp to configured range to protect ESC
    if (value < _min_us) value = _min_us;
    if (value > _max_us) value = _max_us;

    // Set the PWM duty cycle
    esp_err_t err = mcpwm_set_duty_in_us(MCPWM_UNIT_1, MCPWM_TIMER_0, MCPWM_OPR_A, value);
    if (err == ESP_OK) {
      _last_us = value;
      ESP_LOGV("EscController", "ESC set to %u µs", value);
    } else {
      ESP_LOGE("EscController", "Failed to set ESC pulse: %s", esp_err_to_name(err));
    }
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
  void setBraking(bool value) {
    _isBraking = value;
    ESP_LOGV("EscController", "Braking: %s", value ? "ON" : "OFF");
  }

  /**
   * Check if ESC is in braking mode
   * @return true if braking
   */
  bool isBraking() const { return _isBraking; }

  /**
   * Set driving state
   * @param value true if driving forward
   */
  void setDriving(bool value) {
    _isDriving = value;
    ESP_LOGV("EscController", "Driving: %s", value ? "ON" : "OFF");
  }

  /**
   * Check if ESC is in driving mode
   * @return true if driving forward
   */
  bool isDriving() const { return _isDriving; }

  /**
   * Set reverse state
   * @param value true if in reverse
   */
  void setInReverse(bool value) {
    _isInReverse = value;
    ESP_LOGV("EscController", "Reverse: %s", value ? "ON" : "OFF");
  }

  /**
   * Check if ESC is in reverse mode
   * @return true if in reverse
   */
  bool isInReverse() const { return _isInReverse; }

  /**
   * Get ESC configuration limits
   * @param min_us Output parameter for minimum pulse width
   * @param max_us Output parameter for maximum pulse width
   * @param neutral_us Output parameter for neutral pulse width
   * @param frequency Output parameter for PWM frequency
   */
  void getConfig(uint16_t* min_us, uint16_t* max_us, uint16_t* neutral_us, uint16_t* frequency) const {
    if (min_us) *min_us = _min_us;
    if (max_us) *max_us = _max_us;
    if (neutral_us) *neutral_us = _neutral_us;
    if (frequency) *frequency = _frequency;
  }

  /**
   * Get ESC configuration as a simple struct
   * @param info Pointer to structure to fill with configuration
   */
  void getConfigInfo(EscConfigInfo* info) const {
    if (info) {
      info->min_us = _min_us;
      info->max_us = _max_us;
      info->neutral_us = _neutral_us;
      info->frequency = _frequency;
    }
  }

private:
  uint16_t _last_us = ESC_DEFAULT_NEUTRAL_US;
  bool _isBraking = false;
  bool _isDriving = false;
  bool _isInReverse = false;

  // Configuration parameters
  uint16_t _min_us = ESC_DEFAULT_MIN_US;
  uint16_t _max_us = ESC_DEFAULT_MAX_US;
  uint16_t _neutral_us = ESC_DEFAULT_NEUTRAL_US;
  uint16_t _frequency = ESC_DEFAULT_FREQUENCY;

  /**
   * Validate ESC configuration parameters
   * @return true if configuration is valid
   */
  bool validateEscConfig() {
    // Check for valid frequency range
    if (_frequency == 0 || _frequency > 400) {
      ESP_LOGE("EscController", "Invalid ESC frequency: %u Hz", _frequency);
      return false;
    }

    // Check for valid pulse width ranges
    if (_min_us >= _max_us) {
      ESP_LOGE("EscController", "Invalid ESC range: min=%u, max=%u", _min_us, _max_us);
      return false;
    }

    if (_neutral_us < _min_us || _neutral_us > _max_us) {
      ESP_LOGE("EscController", "Invalid ESC neutral: %u (min=%u, max=%u)",
               _neutral_us, _min_us, _max_us);
      return false;
    }

    // Check for reasonable ESC range
    uint16_t range = _max_us - _min_us;
    if (range < 800) {  // Too narrow for ESC
      ESP_LOGW("EscController", "Narrow ESC range: %u µs (recommended >= 800µs)", range);
    }

    if (range > 2000) {  // Very wide range
      ESP_LOGW("EscController", "Wide ESC range: %u µs", range);
    }

    return true;
  }
};