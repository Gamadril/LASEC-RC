#pragma once

#include "board_config.h"
#include "driver/mcpwm.h"
#include "esp_log.h"

extern long map(long x, long in_min, long in_max, long out_min, long out_max);

// Servo timing constants
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_NEUTRAL_US 1500

class ServoController {
public:
  ServoController() = default;

  /**
   * Initialize servo controller with configuration
   * @param steeringServoConfig Pointer to steering servo configuration
   * @param shiftingServoConfig Pointer to shifting servo configuration
   * @return true if initialization successful, false otherwise
   */
  bool init(ServoConfig *steeringServoConfig, ServoConfig *shiftingServoConfig) {
    // Validate input parameters
    if (steeringServoConfig == nullptr || shiftingServoConfig == nullptr) {
      ESP_LOGE("ServoController", "Invalid servo configuration pointers");
      return false;
    }

    // Validate configuration values
    if (!validateServoConfig(steeringServoConfig) || !validateServoConfig(shiftingServoConfig)) {
      ESP_LOGE("ServoController", "Invalid servo configuration values");
      return false;
    }

    this->shiftingServoConfig = shiftingServoConfig;
    this->steeringServoConfig = steeringServoConfig;

    esp_err_t err;

    // Initialize steering servo GPIO
    err = mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, PIN_SERVO_STEERING);
    if (err != ESP_OK) {
      ESP_LOGE("ServoController", "Failed to initialize steering servo GPIO: %s", esp_err_to_name(err));
      return false;
    }

    // Initialize shifting servo GPIO
    err = mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, PIN_SERVO_SHIFTING);
    if (err != ESP_OK) {
      ESP_LOGE("ServoController", "Failed to initialize shifting servo GPIO: %s", esp_err_to_name(err));
      return false;
    }

    // Configure steering servo PWM
    mcpwm_config_t pwm_config_steering = {
        .frequency = steeringServoConfig->frequency,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };

    // Configure shifting servo PWM
    mcpwm_config_t pwm_config_shifting = {
        .frequency = shiftingServoConfig->frequency,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .duty_mode = MCPWM_DUTY_MODE_0,
        .counter_mode = MCPWM_UP_COUNTER,
    };

    // Initialize MCPWM units
    err = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config_steering);
    if (err != ESP_OK) {
      ESP_LOGE("ServoController", "Failed to initialize steering MCPWM: %s", esp_err_to_name(err));
      return false;
    }

    err = mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_1, &pwm_config_shifting);
    if (err != ESP_OK) {
      ESP_LOGE("ServoController", "Failed to initialize shifting MCPWM: %s", esp_err_to_name(err));
      return false;
    }

    // Set initial positions
    set_steering_us(steeringServoConfig->neutral);
    set_shifting_us(shiftingServoConfig->neutral);

    ESP_LOGI("ServoController", "Servo controller initialized successfully");
    return true;
  }

  /**
   * Set steering servo position in microseconds
   * @param value Input value (1000-2000µs)
   */
  void set_steering_us(uint16_t value) {
    static uint16_t lastValue = 0xFFFF;

    // Early return if no change or invalid input
    if (value == lastValue || value < SERVO_MIN_US || value > SERVO_MAX_US) {
      return;
    }

    // Map input range to servo-specific range
    uint16_t mappedValue;
    if (value < SERVO_NEUTRAL_US) {
      mappedValue = map(value, SERVO_MIN_US, SERVO_NEUTRAL_US,
                        steeringServoConfig->min, steeringServoConfig->neutral);
    } else if (value > SERVO_NEUTRAL_US) {
      mappedValue = map(value, SERVO_NEUTRAL_US, SERVO_MAX_US,
                        steeringServoConfig->neutral, steeringServoConfig->max);
    } else {
      mappedValue = steeringServoConfig->neutral;
    }

    // Set the PWM duty cycle
    esp_err_t err = mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, mappedValue);
    if (err == ESP_OK) {
      lastValue = value;
    } else {
      ESP_LOGE("ServoController", "Failed to set steering servo: %s", esp_err_to_name(err));
    }
  }

  /**
   * Get current steering servo position in microseconds
   * @return Current position or neutral value on error
   */
  uint16_t get_steering_us() {
    float duty_percent = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A);
    uint32_t freq_hz = mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_0);

    // Prevent division by zero
    if (freq_hz == 0) {
      ESP_LOGE("ServoController", "Invalid steering servo frequency");
      return steeringServoConfig ? steeringServoConfig->neutral : SERVO_NEUTRAL_US;
    }

    float period_us = 1000000.0f / freq_hz;
    return static_cast<uint16_t>((duty_percent / 100.0f) * period_us);
  }

  /**
   * Set shifting servo position in microseconds
   * @param value Input value (1000-2000µs)
   */
  void set_shifting_us(uint16_t value) {
    static uint16_t lastValue = 0xFFFF;

    // Early return if no change or invalid input
    if (value == lastValue || value < SERVO_MIN_US || value > SERVO_MAX_US) {
      return;
    }

    // Map input range to servo-specific range
    uint16_t mappedValue;
    if (value < SERVO_NEUTRAL_US) {
      mappedValue = map(value, SERVO_MIN_US, SERVO_NEUTRAL_US,
                        shiftingServoConfig->min, shiftingServoConfig->neutral);
    } else if (value > SERVO_NEUTRAL_US) {
      mappedValue = map(value, SERVO_NEUTRAL_US, SERVO_MAX_US,
                        shiftingServoConfig->neutral, shiftingServoConfig->max);
    } else {
      mappedValue = shiftingServoConfig->neutral;
    }

    // Set the PWM duty cycle
    esp_err_t err = mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A, mappedValue);
    if (err == ESP_OK) {
      lastValue = value;
    } else {
      ESP_LOGE("ServoController", "Failed to set shifting servo: %s", esp_err_to_name(err));
    }
  }

  /**
   * Get current shifting servo position in microseconds
   * @return Current position or neutral value on error
   */
  uint16_t get_shifting_us() {
    float duty_percent = mcpwm_get_duty(MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM_OPR_A);
    uint32_t freq_hz = mcpwm_get_frequency(MCPWM_UNIT_0, MCPWM_TIMER_1);

    // Prevent division by zero
    if (freq_hz == 0) {
      ESP_LOGE("ServoController", "Invalid shifting servo frequency");
      return shiftingServoConfig ? shiftingServoConfig->neutral : SERVO_NEUTRAL_US;
    }

    float period_us = 1000000.0f / freq_hz;
    return static_cast<uint16_t>((duty_percent / 100.0f) * period_us);
  }

private:
  ServoConfig *steeringServoConfig;
  ServoConfig *shiftingServoConfig;

  /**
   * Validate servo configuration parameters
   * @param config Pointer to servo configuration to validate
   * @return true if configuration is valid
   */
  bool validateServoConfig(ServoConfig *config) {
    if (config == nullptr) {
      return false;
    }

    // Check for valid frequency range
    if (config->frequency == 0) {
      ESP_LOGE("ServoController", "Invalid servo frequency: %u", config->frequency);
      return false;
    }
    // Note: config->frequency is uint8_t, so values are inherently valid (0-255)

    // Check for valid pulse width ranges
    if (config->min >= config->max) {
      ESP_LOGE("ServoController", "Invalid servo range: min=%u, max=%u", config->min, config->max);
      return false;
    }

    if (config->neutral < config->min || config->neutral > config->max) {
      ESP_LOGE("ServoController", "Invalid servo neutral: %u (min=%u, max=%u)",
               config->neutral, config->min, config->max);
      return false;
    }

    // Check for reasonable servo range
    uint16_t range = config->max - config->min;
    if (range < 500 || range > 1500) {  // Too narrow or too wide
      ESP_LOGW("ServoController", "Unusual servo range: %u µs", range);
    }

    return true;
  }
};
