#pragma once

#include "board_config.hpp"
#if defined(ESP_PLATFORM)
#include "driver/mcpwm_prelude.h"
#endif
#include "utils.hpp"

// map() is defined inline in utils.hpp

// Servo timing constants
#define SERVO_MIN_US 1000
#define SERVO_MAX_US 2000
#define SERVO_NEUTRAL_US 1500

// 1 tick = 1 microsecond, so compare values map 1:1 to microseconds
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000

class ServoController {
public:
  ServoController() = default;

  /**
   * Initialize servo controller with configuration
   * @param config Pointer to configuration
   * @return true if initialization successful, false otherwise
   */
  bool init(Config *config) {
    // Validate input parameters
    if (config == nullptr) {
      ESP_LOGE(TAG, "Invalid configuration pointer");
      return false;
    }

    this->_config = config;
    fprintf(stderr, "ServoController::init: this=%p, _config set to %p\n", (void *)this, (void *)_config);

#if defined(ESP_PLATFORM)
    if (!init_channel(PIN_SERVO_STEERING, config->steeringServo.frequency, config->steeringServo.neutral,
                       &_steeringTimer, &_steeringOper, &_steeringComparator, &_steeringGenerator)) {
      ESP_LOGE(TAG, "Failed to initialize steering servo channel");
      return false;
    }

    if (!init_channel(PIN_SERVO_SHIFTING, config->shiftingServo.frequency, config->shiftingServo.neutral,
                       &_shiftingTimer, &_shiftingOper, &_shiftingComparator, &_shiftingGenerator)) {
      ESP_LOGE(TAG, "Failed to initialize shifting servo channel");
      return false;
    }

    _steeringLastUs = config->steeringServo.neutral;
    _shiftingLastUs = config->shiftingServo.neutral;
#endif

    ESP_LOGI(TAG, "Servo controller initialized successfully");
    return true;
  }

  /**
   * Set steering servo position in microseconds
   * @param value Input value (1000-2000µs)
   */
  void set_steering_us(uint16_t value) {
    static uint16_t lastValue = 0xFFFF;

    if (value == lastValue) {
      return;
    }

    // Map input range to servo-specific range
    uint16_t mappedValue;
    if (value < SERVO_NEUTRAL_US) {
      mappedValue =
          map(value, SERVO_MIN_US, SERVO_NEUTRAL_US, _config->steeringServo.min, _config->steeringServo.neutral);
    } else if (value > SERVO_NEUTRAL_US) {
      mappedValue =
          map(value, SERVO_NEUTRAL_US, SERVO_MAX_US, _config->steeringServo.neutral, _config->steeringServo.max);
    } else {
      mappedValue = _config->steeringServo.neutral;
    }

#if defined(ESP_PLATFORM)
    // Update the comparator's compare value (in timer ticks == microseconds here)
    esp_err_t err = mcpwm_comparator_set_compare_value(_steeringComparator, mappedValue);
    if (err == ESP_OK) {
      lastValue = value;
      _steeringLastUs = mappedValue;
    } else {
      ESP_LOGE(TAG, "Failed to set steering servo: %s", esp_err_to_name(err));
    }
#endif
  }

  /**
   * Get current steering servo position in microseconds
   * @return Current position or neutral value on error
   */
  uint16_t get_steering_us() {
#if defined(ESP_PLATFORM)
    // The new MCPWM driver does not expose a getter for the compare value,
    // so the last value written is tracked and returned instead.
    return _steeringLastUs;
#else
    return _config->steeringServo.neutral;
#endif
  }

  /**
   * Set shifting servo position in microseconds
   * @param value Input value (1000-2000µs)
   */
  void set_shifting_us(uint16_t value) {
    static uint16_t lastValue = 0xFFFF;

    if (value == lastValue) {
      return;
    }

    if (_config == nullptr) {
      return;
    }

    // Map input range to servo-specific range
    uint16_t mappedValue;
    if (value < _config->shiftingServo.neutral) {
      mappedValue =
          map(value, SERVO_MIN_US, SERVO_NEUTRAL_US, _config->shiftingServo.min, _config->shiftingServo.neutral);
    } else if (value > SERVO_NEUTRAL_US) {
      mappedValue =
          map(value, SERVO_NEUTRAL_US, SERVO_MAX_US, _config->shiftingServo.neutral, _config->shiftingServo.max);
    } else {
      mappedValue = _config->shiftingServo.neutral;
    }

#if defined(ESP_PLATFORM)
    // Update the comparator's compare value (in timer ticks == microseconds here)
    esp_err_t err = mcpwm_comparator_set_compare_value(_shiftingComparator, mappedValue);
    if (err == ESP_OK) {
      lastValue = value;
      _shiftingLastUs = mappedValue;
    } else {
      ESP_LOGE(TAG, "Failed to set shifting servo: %s", esp_err_to_name(err));
    }
#endif
  }

  /**
   * Get current shifting servo position in microseconds
   * @return Current position or neutral value on error
   */
  uint16_t get_shifting_us() {
#if defined(ESP_PLATFORM)
    // The new MCPWM driver does not expose a getter for the compare value,
    // so the last value written is tracked and returned instead.
    return _shiftingLastUs;
#else
    return _config->shiftingServo.neutral;
#endif
  }

private:
#if defined(ESP_PLATFORM)
  /**
   * Bring up one MCPWM timer/operator/comparator/generator chain on a given GPIO.
   * @param gpio GPIO number to drive with the PWM signal
   * @param frequency PWM frequency in Hz (used to derive the timer period)
   * @param neutral_us Initial pulse width in microseconds
   * @param timer Out: created timer handle
   * @param oper Out: created operator handle
   * @param comparator Out: created comparator handle
   * @param generator Out: created generator handle
   * @return true on success, false otherwise
   */
  bool init_channel(int gpio, uint32_t frequency, uint16_t neutral_us, mcpwm_timer_handle_t *timer,
                     mcpwm_oper_handle_t *oper, mcpwm_cmpr_handle_t *comparator, mcpwm_gen_handle_t *generator) {
    if (frequency == 0) {
      ESP_LOGE(TAG, "Invalid servo frequency");
      return false;
    }

    mcpwm_timer_config_t timer_config = {};
    timer_config.group_id = 0;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP;
    timer_config.period_ticks = SERVO_TIMEBASE_RESOLUTION_HZ / frequency;

    esp_err_t err = mcpwm_new_timer(&timer_config, timer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_new_timer failed: %s", esp_err_to_name(err));
      return false;
    }

    mcpwm_operator_config_t operator_config = {};
    operator_config.group_id = 0;

    err = mcpwm_new_operator(&operator_config, oper);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_new_operator failed: %s", esp_err_to_name(err));
      return false;
    }

    err = mcpwm_operator_connect_timer(*oper, *timer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_operator_connect_timer failed: %s", esp_err_to_name(err));
      return false;
    }

    mcpwm_comparator_config_t comparator_config = {};
    comparator_config.flags.update_cmp_on_tez = true;

    err = mcpwm_new_comparator(*oper, &comparator_config, comparator);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_new_comparator failed: %s", esp_err_to_name(err));
      return false;
    }

    mcpwm_generator_config_t generator_config = {};
    generator_config.gen_gpio_num = gpio;

    err = mcpwm_new_generator(*oper, &generator_config, generator);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_new_generator failed: %s", esp_err_to_name(err));
      return false;
    }

    err = mcpwm_comparator_set_compare_value(*comparator, neutral_us);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_comparator_set_compare_value failed: %s", esp_err_to_name(err));
      return false;
    }

    // Go high at the start of the timer period...
    err = mcpwm_generator_set_action_on_timer_event(
        *generator, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH));
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_generator_set_action_on_timer_event failed: %s", esp_err_to_name(err));
      return false;
    }

    // ...and go low once the compare value is reached
    err = mcpwm_generator_set_action_on_compare_event(
        *generator, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, *comparator, MCPWM_GEN_ACTION_LOW));
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_generator_set_action_on_compare_event failed: %s", esp_err_to_name(err));
      return false;
    }

    err = mcpwm_timer_enable(*timer);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_timer_enable failed: %s", esp_err_to_name(err));
      return false;
    }

    err = mcpwm_timer_start_stop(*timer, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "mcpwm_timer_start_stop failed: %s", esp_err_to_name(err));
      return false;
    }

    return true;
  }
#endif

  static inline const char *TAG = "SRV";
  Config *_config = nullptr;

#if defined(ESP_PLATFORM)
  mcpwm_timer_handle_t _steeringTimer = nullptr;
  mcpwm_oper_handle_t _steeringOper = nullptr;
  mcpwm_cmpr_handle_t _steeringComparator = nullptr;
  mcpwm_gen_handle_t _steeringGenerator = nullptr;

  mcpwm_timer_handle_t _shiftingTimer = nullptr;
  mcpwm_oper_handle_t _shiftingOper = nullptr;
  mcpwm_cmpr_handle_t _shiftingComparator = nullptr;
  mcpwm_gen_handle_t _shiftingGenerator = nullptr;

  uint16_t _steeringLastUs = SERVO_NEUTRAL_US;
  uint16_t _shiftingLastUs = SERVO_NEUTRAL_US;
#endif
};