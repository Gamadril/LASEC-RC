// #include "argtable3/argtable3.h"
// #include "ble_constroller.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// ESP-IDF equivalents for Arduino functions
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define MAX_SUMD_CHANNELS 16

#define VERSION 1.0f

#include "curves.h" // Nonlinear throttle curve arrays

#include "config_handler.h"
// #include "dashboard.h"
#include "esc_controller.h"
#include "lights_controller.h"
#include "model_state.h"
#include "receiver_controller.h"
// #include "rgb_controller.h"
#include "serialport_controller.h"
#include "servo_controller.h"
#include "sound_controller_i2s.h"

// Debug counters definition
volatile uint32_t debug_i2s_errors = 0;
volatile uint32_t debug_i2s_samples = 0;
volatile uint32_t debug_audio_clips = 0;
volatile uint32_t debug_timer_updates = 0;
volatile uint32_t debug_knock_triggers = 0;
volatile uint32_t debug_large_samples = 0;
volatile int16_t debug_max_left = 0;
volatile int16_t debug_max_right = 0;
#include "vehicle.h"

// Engine mix shared between ISRs
volatile int32_t engine_mix_value = 0;

void Task1code(void *pvParameters);
void user_main_task(void *pvParameters);
void engineMassSimulation();
void esc();
void mapThrottle();
void gearboxDetection();
void onTurnIndicator(bool left, bool right);
void onReceiverMessage(uint16_t *channels, uint8_t channels_count, bool failsafe);
void onSerialMessage(SerialMessage &msg);

ConfigHandler configHandler;
ModelState state;
// Dashboard dashboard;
EscController escController;
ReceiverController receiverController(UART_NUM_2);
ServoController servoController;
LightsController lightsController;
SerialPortController serialPortController;
SoundController soundController;
// RgbController rgbController;

RcConfig *rcConfig;
ServoConfig *steeringServoConfig;
ServoConfig *shiftingServoConfig;
Vehicle *vehicle;
Config *config;

// Channel values array
uint16_t channel_values[MAX_SUMD_CHANNELS];

TaskHandle_t Task1;
uint16_t speedLimit = 500;
bool gearUpShiftingPulse = false;
bool gearDownShiftingPulse = false;

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  const long dividend = out_max - out_min;
  const long divisor = in_max - in_min;
  const long delta = x - in_min;
  if (divisor == 0) {
    ESP_LOGE("TOOL", "Invalid map input range, min == max");
    return -1;
  }
  return (delta * dividend + (divisor / 2)) / divisor + out_min;
}

unsigned long IRAM_ATTR micros() {
  return (unsigned long)(esp_timer_get_time());
}

unsigned long IRAM_ATTR millis() {
  return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

uint16_t getThrottleValue() {
  // return channel_values[rcConfig->channel_map[CH_THROTTLE] - 1];
  return channel_values[CH_THROTTLE - 1];
}

uint16_t getSteeringValue() {
  // return channel_values[rcConfig->channel_map[CH_STEERING] - 1];
  return channel_values[CH_STEERING - 1];
}

uint16_t getShiftingValue() {
  // return channel_values[rcConfig->channel_map[CH_SHIFTING] - 1];
  return channel_values[CH_SHIFTING - 1];
}

bool isGearIn() {
  // return channel_values[rcConfig->channel_map[CH_SHIFTING] - 1] > 1500;
  return channel_values[CH_GEAR_CLUTCH - 1] > 1500;
}

bool isHazardOn() {
  return channel_values[8] > 1500;
}

uint16_t getRgbValue() {
  uint16_t value = channel_values[CH_RGB - 1];
  return map(value, 1000, 2000, 0, 255);
}

bool shouldSetLeftTurnIndicator() {
  uint16_t value = getSteeringValue();
  return (value > (steeringServoConfig->neutral + vehicle->indicatorOnThreshold)) ||
         state.isHazard();
}

bool shouldSetRightTurnIndicator() {
  uint16_t value = getSteeringValue();
  return (value < (steeringServoConfig->neutral - vehicle->indicatorOnThreshold)) ||
         state.isHazard();
}

bool shouldSetLowBeamLight() {
  return false;
}

bool shouldSetHighBeamLight() {
  return false;
}

bool shouldSetBrakeLight() {
  return false;
}

bool shouldSetReverseLight() {
  return false;
}

bool shouldSetParkingLight() {
  return false;
}

bool shouldSetDaytimelight() {
  return false;
}

bool shouldSetFogLight() {
  return false;
}

bool shouldTriggerHorn() {
  return channel_values[CH_HORN - 1] > 1600;
}

void onTurnIndicator(bool left, bool right) {
  soundController.onTurnSignal(left || right);

  // dashboard.setLeftIndicator(left);
  // dashboard.setRightIndicator(right);
}

void onReceiverMessage(uint16_t *channels, uint8_t channels_count, bool failsafe) {
  state.setFailsafe(failsafe);
  memcpy(channel_values, channels, MAX_SUMD_CHANNELS * sizeof(uint16_t));
}

void onConfigReceived(const Config *config) {
  ESP_LOGI("MAIN", "Received config from browser:");
  ESP_LOGI("MAIN", "Model name: %.40s", config->model_name);
  ESP_LOGI("MAIN", "Volume: %d%%", config->volume);
  ESP_LOGI("MAIN", "Steering servo min/max: %d/%d", config->steeringServo.min,
           config->steeringServo.max);

  // Update local config
  configHandler.setConfig(const_cast<Config *>(config));
  configHandler.save();

  ESP_LOGI("MAIN", "Config saved to NVS");
}

Config *onConfigRequested() {
  ESP_LOGI("MAIN", "Browser requested current config");
  return configHandler.getConfig();
}

/*
void onSerialMessage(SerialMessage &msg) {
  state.setMode(CONFIG);

  if (msg.type == MSG_PING) {
    serialPortController.send(msg);
  } else if (msg.type == MSG_STATE_GET) {
    SerialMessage resp;
    resp.type = MSG_STATE_DATA;
    resp.payload_size = sizeof(State);
    resp.payload = (uint8_t *)state.getState();
    serialPortController.send(resp);
  } else if (msg.type == MSG_CONFIG_GET) {
    SerialMessage resp;
    Config *cfg = configHandler.getConfig();
    resp.type = MSG_CONFIG_DATA;
    resp.payload_size = sizeof(Config);
    resp.payload = (uint8_t *)cfg;
    serialPortController.send(resp);
  } else if (msg.type == MSG_CONFIG_SET) {
    configHandler.setConfig((Config *)msg.payload);
  } else if (msg.type == MSG_CONFIG_SAVE) {
    soundController.deinit();
    vTaskDelete(Task1);
    configHandler.save();
    // soundController.init(&state, configHandler.getVehicle(), configHandler.getConfig());
    // xTaskCreatePinnedToCore(Task1code, "Task1", 100000, NULL, 1, &Task1, 0);
    //  } else if (msg.type == MSG_RGB_COLOUR_GET) {
    //    SerialMessage resp;
    //    CRGB rgb = rgbController.getColour(0);
    //    resp.type = MSG_RGB_COLOUR_DATA;
    //    resp.payload_size = sizeof(CRGB);
    //    resp.payload = rgb.raw;
    //    serialPortController.send(resp);
    //  } else if (msg.type == MSG_RGB_COLOUR_SET) {
    //    rgbController.setColour(0, msg.payload[0], msg.payload[1], msg.payload[2]);
    //    rgbController.show();
    //  } else if (msg.type == MSG_RGB_BRIGHTNESS_GET) {
    //    SerialMessage resp;
    //    uint8_t brightness = rgbController.getBrightness();
    //    resp.type = MSG_RGB_BRIGHTNESS_DATA;
    //    resp.payload_size = 1;
    //    resp.payload = &brightness;
    //    serialPortController.send(resp);
    //  } else if (msg.type == MSG_RGB_BRIGHTNESS_SET) {
    //    rgbController.setBrightness(msg.payload[0]);
    //    rgbController.show();
  } else if (msg.type == MSG_VEHICLE_SET) {
    memcpy(vehicle, msg.payload, sizeof(Vehicle));
  } else if (msg.type == MSG_SERVO_STEERING_GET) {
    SerialMessage resp;
    uint16_t value = servoController.get_steering_us();
    uint8_t payload[2];
    payload[0] = (value >> 8) & 0xFF;
    payload[1] = value & 0xFF;
    resp.type = MSG_SERVO_STEERING_DATA;
    resp.payload_size = 2;
    resp.payload = payload;
    serialPortController.send(resp);
  } else if (msg.type == MSG_SERVO_STEERING_SET) {
    servoController.set_steering_us(msg.payload[0] << 8 | msg.payload[1]);
  } else if (msg.type == MSG_SERVO_SHIFTING_GET) {
    SerialMessage resp;
    uint16_t value = servoController.get_shifting_us();
    uint8_t payload[2];
    payload[0] = (value >> 8) & 0xFF;
    payload[1] = value & 0xFF;
    resp.type = MSG_SERVO_SHIFTING_DATA;
    resp.payload_size = 2;
    resp.payload = payload;
    serialPortController.send(resp);
  } else if (msg.type == MSG_SERVO_SHIFTING_SET) {
    servoController.set_shifting_us(msg.payload[0] << 8 | msg.payload[1]);
  } else if (msg.type == MSG_ESC_GET) {
    SerialMessage resp;
    uint16_t value = escController.get_esc_us();
    uint8_t payload[2];
    payload[0] = (value >> 8) & 0xFF;
    payload[1] = value & 0xFF;
    resp.type = MSG_ESC_DATA;
    resp.payload_size = 2;
    resp.payload = payload;
    serialPortController.send(resp);
  } else if (msg.type == MSG_ESC_SET) {
    escController.set_esc_us(msg.payload[0] << 8 | msg.payload[1]);
  }
}
*/

extern "C" void app_main() {
  // Initialize NVS first (required for BLE/WiFi calibration)
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  ESP_LOGI("MAIN", "NVS initialized");

  // Disable task watchdog for idle tasks to prevent false alarms
  esp_task_wdt_deinit();

  memset(channel_values, 0, sizeof(channel_values));

  configHandler.init();

  // Initialize BLE with callbacks
  // BLEController::onConfigReceived = onConfigReceived;
  // BLEController::onConfigRequested = onConfigRequested;
  // BLEController::init("LASEC-RC-Actros");

  lightsController.init(onTurnIndicator);
  // serialPortController.init(SERIAL_BAUD_RATE, onSerialMessage);

  rcConfig = configHandler.getRcConfig();
  config = configHandler.getConfig();
  vehicle = configHandler.getVehicle();
  steeringServoConfig = configHandler.getSteeringServoConfig();
  shiftingServoConfig = configHandler.getShiftingServoConfig();

  // if (configHandler.hasRgbLed()) {
  //   rgbController.init(1);
  //   rgbController.setBrightness(configHandler.getRGBBrightness());
  //   rgbController.setColour(0, configHandler.getRGBColour());
  //   rgbController.show();
  // }

  if (configHandler.hasDashboard()) {
    // dashboard.init();
  }

  // Initialize controllers with error checking
  if (!escController.init()) {
    ESP_LOGE("MAIN", "ESC controller initialization failed");
    // Continue anyway, ESC might work with defaults
  }

  if (!servoController.init(steeringServoConfig, shiftingServoConfig)) {
    ESP_LOGE("MAIN", "Servo controller initialization failed");
    // This is critical - servo control is essential
    vTaskDelay(pdMS_TO_TICKS(5000)); // Give user time to see error
  }

  receiverController.init(onReceiverMessage);
  soundController.init(&state, vehicle, config);

  BaseType_t rc;
  // Create user main task first (core 1), then Task1 (core 0) to avoid starving app_main
  rc = xTaskCreatePinnedToCore(user_main_task, "user_main_task", 8192, NULL, 1, NULL, 1);
  if (rc != pdPASS) {
    ESP_LOGE("MAIN", "main_task create failed rc=%d", rc);
  }

  rc = xTaskCreatePinnedToCore(Task1code, "Task1", 8192, NULL, 2, &Task1, 0);
  if (rc != pdPASS) {
    ESP_LOGE("MAIN", "Task1 create failed rc=%d", rc);
  }
}

// Main task function (replaces Arduino loop)
void user_main_task(void *pvParameters) {
  while (1) {
    // Small delay to prevent task from consuming too much CPU
    vTaskDelay(pdMS_TO_TICKS(1));

    // Process serial messages in main loop, not in ISR
    // serialPortController.processMessages();

    if (state.getMode() == CONFIG) {
      // config mode, get all values from serial port
    } else {
      mapThrottle();

      // play mode, get real values
      uint16_t value;

      state.setHazard(isHazardOn());

      soundController.onHorn(shouldTriggerHorn());

      value = getThrottleValue();

      if (state.getEngineState() == OFF && value > 1700) {
        state.setEngineState(STARTING);
        soundController.onIgnition(true);
      } else if (state.getEngineState() == RUNNING && state.getDriveState() == STANDING &&
                 value < 1300) {
        soundController.onIgnition(false);
      }

      // Servo control
      value = getSteeringValue();
      servoController.set_steering_us(value);

      value = getShiftingValue();
      servoController.set_shifting_us(value);

      lightsController.setLeftIndicator(shouldSetLeftTurnIndicator() ? LIGHT_ON : LIGHT_OFF);
      lightsController.setRightIndicator(shouldSetRightTurnIndicator() ? LIGHT_ON : LIGHT_OFF);
      lightsController.setLowBeamLight(shouldSetLowBeamLight() ? LIGHT_ON : LIGHT_OFF);
      lightsController.setHighBeamLight(shouldSetHighBeamLight() ? LIGHT_ON : LIGHT_OFF);
      lightsController.setFogLight(shouldSetFogLight() ? LIGHT_ON : LIGHT_OFF);
      lightsController.setDaytimeLight(shouldSetDaytimelight() ? LIGHT_ON : LIGHT_OFF);

      if (configHandler.hasRgbLed()) {
        uint8_t hue = getRgbValue();
        // rgbController.setColour(0, CHSV(hue, hue < 255 ? 255 : 0, hue > 0 ? 255 : 0));
      }
    }

    // Check for receiver timeout and trigger failsafe if no signal
    receiverController.checkTimeout();

    // Small delay to prevent task from consuming too much CPU
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

void dacOffsetFade() {
  // I2S version doesn't need DAC offset fading - no-op
}

// 1st MAIN TASK, RUNNING ON CORE 0 (Interrupts are running on this core as well)
void Task1code(void *pvParameters) {
  while (true) {
    dacOffsetFade();
    engineMassSimulation();
    gearboxDetection();
    esc();
  }
}

void engineMassSimulation() {
  static int32_t targetRpm = 0; // The engine RPM target
  static unsigned long throtMillis = 0;
  uint8_t timeBase = 2;
  static bool clutchDisengaged = false;

  if (millis() - throtMillis > timeBase) { // Every 2 or 6ms
    throtMillis = millis();

    // Virtual clutch
    if ((state.getSpeed() < vehicle->clutchEngagingPoint && state.getRPM() < 250) ||
        state.getGearShift() != NOT || state.getGear() == 0) {
      clutchDisengaged = true;
    } else {
      clutchDisengaged = false;
    }

    // Transmissions
    // Manual transmission ----
    if (clutchDisengaged) { // Clutch disengaged: Engine revving allowed
      targetRpm = reMap(curveLinear, state.getThrottle());
    } else { // Clutch engaged: Engine rpm synchronized with ESC power (speed)
      targetRpm = reMap(curveLinear, state.getSpeed());
    }

    // Engine RPM
    if (escController.isBraking() && state.getSpeed() < vehicle->clutchEngagingPoint) {
      targetRpm = 0; // keep engine @idle rpm, if braking at very low speed
    }

    // Accelerate engine
    if (targetRpm > (state.getRPM() + vehicle->acc) && (state.getRPM() + vehicle->acc) < MAX_RPM &&
        state.getEngineState() == RUNNING) {
      if (!state.isAirBrake()) { // No acceleration, if brake release noise still playing
        if (state.getGearShift() == DOWN) {
          state.setRPM(state.getRPM() +
                       vehicle->acc / 2); // less aggressive rpm rise while downshifting
        } else {
          state.setRPM(state.getRPM() + vehicle->acc);
        }
      }
    }

    // Decelerate engine
    if (targetRpm < state.getRPM()) {
      int newRpm = (int)state.getRPM() - (int)vehicle->dec;
      if (newRpm < 0) {
        state.setRPM(MIN_RPM);
      } else {
        state.setRPM(newRpm);
      }
    }

    // Speed (sample rate) output
    soundController.onRpmChange();
  }
}

void esc() {
  static int32_t escPulseWidth = 1500;
  static uint32_t escSignal = 0;
  static uint64_t escMillis = 0;
  static int8_t pulse = 0;    // -1 = reverse, 0 = neutral, 1 = forward
  static int8_t escPulse = 0; // -1 = reverse, 0 = neutral, 1 = forward
  static int8_t driveRampRate = 0;
  static int8_t driveRampGain = 0;
  static int8_t brakeRampRate = 0;
  uint8_t escRampTime = 0;

  switch (state.getGear()) {
  case 1:
    escRampTime = vehicle->escRampTimeFirstGear;
    break;
  case 2:
    escRampTime = vehicle->escRampTimeSecondGear;
    break;
  case 3:
    escRampTime = vehicle->escRampTimeThirdGear;
    break;
  default:
    escRampTime = 20;
  }

  if (millis() - escMillis > escRampTime) { // About very 20 - 75ms
    escMillis = millis();

    if (state.isFailsafe()) {
      brakeRampRate = vehicle->escBrakeSteps;
      driveRampRate = vehicle->escBrakeSteps;
    } else {
      // calulate throttle dependent brake & acceleration steps
      brakeRampRate = map(state.getThrottle(), 0, 500, 1, vehicle->escBrakeSteps);
      driveRampRate = map(state.getThrottle(), 0, 500, 1, vehicle->escAccelerationSteps);
    }

    uint16_t throttle = getThrottleValue();

    if (throttle > config->escConfig.neutral && throttle < config->escConfig.max) {
      pulse = 1; // 1 = Forward
    } else if (throttle < config->escConfig.neutral && throttle > config->escConfig.min) {
      pulse = -1; // -1 = Backwards
    } else {
      pulse = 0; // 0 = Neutral
    }

    if (escPulseWidth > config->escConfig.neutral && escPulseWidth < config->escConfig.max) {
      escPulse = 1; // 1 = Forward
    } else if (escPulseWidth < config->escConfig.neutral && escPulseWidth > config->escConfig.min) {
      escPulse = -1; // -1 = Backwards
    } else {
      escPulse = 0; // 0 = Neutral
    }

    // Drive state state machine
    // **********************************************************************************
    switch (state.getDriveState()) {
    case STANDING:
      escController.setBraking(false);
      escController.setDriving(false);
      escController.setInReverse(false);
      escPulseWidth = config->escConfig.neutral; // ESC to neutral position

      if (pulse == 1 && state.getEngineState() == RUNNING && state.getGear() != 0) {
        state.setDriveState(DRIVING_FORWARD);
      } else if (pulse == -1 && state.getEngineState() == RUNNING && state.getGear() != 0) {
        state.setDriveState(DRIVING_BACKWARD);
      }
      break;
    case DRIVING_FORWARD:
      escController.setBraking(false);
      escController.setDriving(true);
      escController.setInReverse(false);

      // If clutch is disengaged (gear == 0), coast to neutral
      if (state.getGear() == 0) {
        if (escPulseWidth > config->escConfig.neutral) {
          escPulseWidth -= 2; // Slow coast down (slower than braking)
        } else {
          escPulseWidth = config->escConfig.neutral;
        }
      }
      // Normal driving with clutch engaged
      else if (escPulseWidth < throttle && state.getSpeed() < speedLimit) {
        if (escPulseWidth >= config->escConfig.neutral) {
          escPulseWidth += (driveRampRate * driveRampGain); // Faster
        } else {
          escPulseWidth = config->escConfig.neutral; // Initial boost
        }
      } else if (escPulseWidth > throttle && escPulseWidth > config->escConfig.neutral) {
        escPulseWidth -= (driveRampRate * driveRampGain); // Slower
      }

      if (gearUpShiftingPulse && vehicle->shiftingAutoThrottle) {
        // lowering RPM, if shifting up transmission
        escPulseWidth -= state.getSpeed() / 4; // Synchronize engine speed
        gearUpShiftingPulse = false;
        escPulseWidth = constrain(escPulseWidth, config->escConfig.neutral, config->escConfig.max);
      } else if (gearDownShiftingPulse && vehicle->shiftingAutoThrottle) {
        // increasing RPM, if shifting down transmission
        escPulseWidth += 50; // Synchronize engine speed
        gearDownShiftingPulse = false;
        escPulseWidth = constrain(escPulseWidth, config->escConfig.neutral, config->escConfig.max);
      }

      if (pulse == -1 && escPulse == 1) {
        state.setDriveState(BRAKING_FORWARD);
      } else if (pulse == -1 && escPulse == 0 && state.getGear() != 0) {
        state.setDriveState(DRIVING_BACKWARD); // Prevents state machine from hanging!
      } else if (escPulse == 0 && (pulse == 0 || state.getGear() == 0)) {
        state.setDriveState(
            STANDING); // Go to standing when stopped and (throttle neutral or clutch off)
      }
      break;

    case BRAKING_FORWARD:
      escController.setBraking(true);
      escController.setDriving(false);
      escController.setInReverse(false);

      if (escPulseWidth > config->escConfig.neutral) {
        escPulseWidth -= brakeRampRate; // brake with variable deceleration
      } else if (escPulseWidth < config->escConfig.neutral) {
        escPulseWidth = config->escConfig.neutral; // Overflow bug prevention!
      }

      if (pulse == 0 && escPulse == 1 && state.getGear() != 0) {
        state.setDriveState(DRIVING_FORWARD);
        state.setAirBrake(true);
      } else if (pulse == 0 && escPulse == 0) {
        state.setDriveState(STANDING);
        state.setAirBrake(true);
      }
      break;

    case DRIVING_BACKWARD:
      escController.setBraking(false);
      escController.setDriving(true);
      escController.setInReverse(true);

      // If clutch is disengaged (gear == 0), coast to neutral
      if (state.getGear() == 0) {
        if (escPulseWidth < config->escConfig.neutral) {
          escPulseWidth += 2; // Slow coast down (slower than braking)
        } else {
          escPulseWidth = config->escConfig.neutral;
        }
      }
      // Normal driving with clutch engaged
      else if (escPulseWidth > throttle && state.getSpeed() < speedLimit) {
        if (escPulseWidth <= config->escConfig.neutral) {
          escPulseWidth -= (driveRampRate * driveRampGain); // Faster
        } else {
          escPulseWidth = config->escConfig.neutral; // Initial boost
        }
      } else if (escPulseWidth < throttle && escPulseWidth < config->escConfig.neutral) {
        escPulseWidth += (driveRampRate * driveRampGain); // Slower
      }

      if (gearUpShiftingPulse && vehicle->shiftingAutoThrottle) {
        // lowering RPM, if shifting up transmission
        escPulseWidth += state.getSpeed() / 4; // Synchronize engine speed
        gearUpShiftingPulse = false;
        escPulseWidth = constrain(escPulseWidth, config->escConfig.min, config->escConfig.neutral);
      } else if (gearDownShiftingPulse && vehicle->shiftingAutoThrottle) {
        // increasing RPM, if shifting down transmission
        escPulseWidth -= 50; // Synchronize engine speed
        gearDownShiftingPulse = false;
        escPulseWidth = constrain(escPulseWidth, config->escConfig.min, config->escConfig.neutral);
      }

      if (pulse == 1 && escPulse == -1) {
        state.setDriveState(BRAKING_BACKWARD);
      } else if (pulse == 1 && escPulse == 0 && state.getGear() != 0) {
        state.setDriveState(DRIVING_FORWARD); // Prevents state machine from hanging!
      } else if (escPulse == 0 && (pulse == 0 || state.getGear() == 0)) {
        state.setDriveState(
            STANDING); // Go to standing when stopped and (throttle neutral or clutch off)
      }
      break;

    case BRAKING_BACKWARD:
      escController.setBraking(true);
      escController.setDriving(false);
      escController.setInReverse(false);

      if (escPulseWidth < config->escConfig.neutral) {
        escPulseWidth += brakeRampRate; // brake with variable deceleration
      } else if (escPulseWidth > config->escConfig.neutral) {
        escPulseWidth = config->escConfig.neutral; // Overflow bug prevention!
      }

      if (pulse == 0 && escPulse == -1 && state.getGear() != 0) {
        state.setDriveState(DRIVING_BACKWARD);
        state.setAirBrake(true);
      } else if (pulse == 0 && escPulse == 0) {
        state.setDriveState(STANDING);
        state.setAirBrake(true);
      }
      break;
    }

    // Gain for drive ramp rate, depending on clutchEngagingPoint
    if (state.getSpeed() < vehicle->clutchEngagingPoint) {
      driveRampGain = 2; // prevent clutch from slipping too much (2)
    } else {
      driveRampGain = 1;
    }

    // ESC control

    escSignal = map(escPulseWidth, config->escConfig.min, config->escConfig.max, 1000, 2000);
    // escSignal = map(escPulseWidthOut, escPulseMax, escPulseMin, 1000, 2000); // direction
    // inversed
    escController.set_esc_us(escSignal);

    // Calculate a speed value from the pulsewidth signal (used as base for engine sound RPM while
    // clutch is engaged)
    if (escPulseWidth > config->escConfig.neutral) {
      state.setSpeed(map(escPulseWidth, config->escConfig.neutral, config->escConfig.max, 0, 500));
    } else if (escPulseWidth < config->escConfig.neutral) {
      state.setSpeed(map(escPulseWidth, config->escConfig.neutral, config->escConfig.min, 0, 500));
    } else {
      state.setSpeed(0);
    }
  }
}

void mapThrottle() {
  // Input is around 1000 - 2000us, output 0-500 for forward and backwards
  uint16_t throttle = getThrottleValue();

  // calculate a throttle value from the pulsewidth signal
  if (throttle > config->escConfig.neutral) {
    state.setThrottle(map(throttle, config->escConfig.neutral, config->escConfig.max, 0, 500));
  } else if (throttle < config->escConfig.neutral) {
    state.setThrottle(map(throttle, config->escConfig.neutral, config->escConfig.min, 0, 500));
  } else {
    state.setThrottle(0);
  }

  // Auto throttle while gear shifting (synchronizing the Tamiya 3 speed gearbox)
  if (state.getDriveState() == DRIVING_FORWARD && vehicle->shiftingAutoThrottle) {
    if (state.getGearShift() == UP) {
      state.setThrottle(0);
    } else if (state.getGearShift() == DOWN) {
      state.setThrottle(500);
    }
  }

  // Auto throttle while gear shifting (synchronizing the Tamiya 3 speed gearbox)
  if (escController.isBraking() && escController.isDriving() && vehicle->shiftingAutoThrottle) {
    if (state.getGearShift() == GearShift::UP) {
      state.setThrottle(0);
    } else if (state.getGearShift() == GearShift::DOWN) {
      state.setThrottle(500);
    }
  }

  soundController.onThrottleChange(state.getThrottle());
}

void gearboxDetection() {
  static uint8_t previousGear = 1;
  static bool previousReverse;
  static unsigned long upShiftingMillis;
  static unsigned long downShiftingMillis;

  if (!isGearIn()) {
    state.setGear(0);
    state.setGearShift(NOT);
    previousGear = 0; // Reset so gear detection works when re-engaged
    // Don't force braking - let the ESC state machine handle coasting with gear == 0
    return;
  }

  // Gear detection
  uint16_t value = getShiftingValue();
  if (value > 1700) {
    state.setGear(3);
  } else if (value < 1300) {
    state.setGear(1);
  } else {
    state.setGear(2);
  }

  // If clutch was just re-engaged (previousGear == 0), update previousGear without triggering shift
  if (previousGear == 0) {
    previousGear = state.getGear();
  }
  // Gear upshifting detection
  else if (state.getGear() > previousGear) {
    state.setGearShift(UP);
    gearUpShiftingPulse = true;
    soundController.shifting = true;
    previousGear = state.getGear();
  }
  // Gear downshifting detection
  else if (state.getGear() < previousGear) {
    state.setGearShift(DOWN);
    gearDownShiftingPulse = true;
    soundController.shifting = true;
    previousGear = state.getGear();
  }

  // Gear upshifting duration
  if (state.getGearShift() != UP) {
    upShiftingMillis = millis();
  }
  if (millis() - upShiftingMillis > 700) {
    state.setGearShift(NOT);
  }

  // Gear downshifting duration
  if (state.getGearShift() != DOWN) {
    downShiftingMillis = millis();
  }
  if (millis() - downShiftingMillis > 300) {
    state.setGearShift(NOT);
  }

  // Reverse gear engaging / disengaging detection
  if (escController.isInReverse() != previousReverse) {
    previousReverse = escController.isInReverse();
    soundController.shifting = true; // Play shifting sound
  }
}