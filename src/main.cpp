// #include "ble_constroller.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "common.hpp"
#include "config_handler.hpp"
// #include "dashboard.h"
#include "esc_controller.hpp"
#include "lights_controller.hpp"
#include "model_state.hpp"
#include "receiver_controller.hpp"
#include "rgb_controller.hpp"
#include "ble_controller.hpp"
#include "esp_spiffs.h"
#include "hal_impl/persistence_fs.hpp"
#include "nvs_flash.h"
#include "hal_impl/sound_output_i2s.hpp"
#include "hal_impl/timer_esp.hpp"
#include "hal_impl/wav_reader_mmap.hpp"
#include "servo_controller.hpp"
#include "signal_handler.hpp"
#include "sound_controller.hpp"
#include "sound_manager.hpp"
#include "vehicle.hpp"

#include "driver/gpio.h"

#define MAX_SUMD_CHANNELS 16
#define VERSION 1.0f

// Engine mix shared between ISRs
volatile int32_t engine_mix_value = 0;

void Task1code(void *pvParameters);
void user_main_task(void *pvParameters);
void engineMassSimulation();
void esc();
void gearboxDetection();
void initNVS();
void mountSPIFFS();

PersistenceFS persistence;
ConfigHandler configHandler(persistence, "/spiffs/config.json");
ModelState state;
// Dashboard dashboard;
EscController escController;
BLEController bleController;
ReceiverController receiverController(UART_NUM_2, PIN_RECEIVER);
ServoController servoController;

// trigger on/off period for turn indicator must be the same as the duration between two tick starts in the correponding
// audio file
LightsController lightsController([]() { return new TimerESP(345); });

SoundOutputI2S soundOutput;
SoundManager soundManager([]() { return new WavReaderMMap(); });
SoundController *soundController = nullptr;

RgbController rgbController;
Config *config;

TaskHandle_t Task1;
uint16_t speedLimit = 500;

const char *TAG = "MAIN";

// uint16_t getRgbValue() {
// uint16_t value = channel_values[CH_RGB - 1];
// return map(value, 1000, 2000, 0, 255);
//}

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

extern "C" void app_main() {
  // Disable task watchdog for idle tasks to prevent false alarms
  esp_task_wdt_deinit();

  // Give serial monitor time to connect before logging starts
  //vTaskDelay(pdMS_TO_TICKS(5000));

      //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    // bit mask of the pins that you want to set, e.g. GPIO18/19
    // GPIO_NUM_13 is the pin number, but gpio_config expects a bitmask.
    io_conf.pin_bit_mask = (1ULL << GPIO_NUM_13);
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

  for (int i = 0; i < 100; ++i) {    
    gpio_set_level(GPIO_NUM_13, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));
    gpio_set_level(GPIO_NUM_13, 0);
    vTaskDelay(pdMS_TO_TICKS(3000));
  }

  // Initialize NVS first (required for BLE/WiFi calibration)
  initNVS();
  mountSPIFFS();
  soundManager.scan("/spiffs");

  configHandler.init();
  // Load persisted config into global pointer
  config = configHandler.getConfig();

  lightsController.init();

  if (!escController.init(config)) {
    ESP_LOGE(TAG, "ESC controller initialization failed");
  }

  if (!servoController.init(config)) {
    ESP_LOGE(TAG, "Servo controller initialization failed");
  }

  // Always init strip so BLE tester can drive it after enabling hasRgbLed
  rgbController.init(1);
  if (configHandler.hasRgbLed()) {
    rgbController.setBrightness(configHandler.getRGBBrightness());
    rgbController.setColour(0, configHandler.getRGBColour());
    rgbController.show();
  }


  if (configHandler.hasDashboard()) {
    // dashboard.init();
  }

  soundController = new SoundController(soundOutput, soundManager.getSounds());
  soundController->init(&state, config);

  setupSignalHandlers(&state, &lightsController, soundController, &escController, &receiverController, &servoController,
                      config);

  receiverController.init();

  bleController.init(state.getState(), &servoController, &configHandler, &rgbController);

  BaseType_t rc;
  // Create user main task first (core 1), then Task1 (core 0) to avoid starving app_main
  rc = xTaskCreatePinnedToCore(user_main_task, "user_main_task", 8192, NULL, 1, NULL, 1);
  if (rc != pdPASS) {
    ESP_LOGE(TAG, "main_task create failed rc=%d", rc);
  }

  rc = xTaskCreatePinnedToCore(Task1code, "Task1", 8192, NULL, 2, &Task1, 0);
  if (rc != pdPASS) {
    ESP_LOGE(TAG, "Task1 create failed rc=%d", rc);
  }
}

// Main task function
void user_main_task(void *pvParameters) {
  while (true) {
    lightsController.setLowBeam(shouldSetLowBeamLight() ? LIGHT_ON : LIGHT_OFF);
    lightsController.setHighBeam(shouldSetHighBeamLight() ? LIGHT_ON : LIGHT_OFF);
    lightsController.setFog(shouldSetFogLight() ? LIGHT_ON : LIGHT_OFF);
    lightsController.setDaytime(shouldSetDaytimelight() ? LIGHT_ON : LIGHT_OFF);

    if (configHandler.hasRgbLed()) {
      // uint8_t hue = getRgbValue();
      //  rgbController.setColour(0, CHSV(hue, hue < 255 ? 255 : 0, hue > 0 ? 255 : 0));
    }

    // Check for receiver timeout and trigger failsafe if no signal
    receiverController.checkTimeout();

    // Small delay to prevent task from consuming too much CPU
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// 1st MAIN TASK, RUNNING ON CORE 0 (Interrupts are running on this core as well)
void Task1code(void *pvParameters) {
  while (true) {
    state.checkGearShiftingStop();
    // Update ESC output first so engine simulation uses the latest speed
    esc();
    engineMassSimulation(config, &state, soundController, &escController);
  }
}

void esc() {
  static int32_t escPulseWidth = config->escConfig.neutral;
  static uint32_t escSignal = 0;
  static uint64_t escMillis = 0;
  static int8_t pulse = 0;    // -1 = reverse, 0 = neutral, 1 = forward
  static int8_t escPulse = 0; // -1 = reverse, 0 = neutral, 1 = forward
  static int8_t driveRampRate = 0;
  static int8_t driveRampGain = 0;
  static int8_t brakeRampRate = 0;
  uint8_t escRampTime = 0;

  // FIXME: state.getGearShift() might be too long compared to previous code with only one pulse enabled and disabled

  switch (state.getGear()) {
    case 1:
      escRampTime = config->vehicle.escRampTimeFirstGear;
      break;
    case 2:
      escRampTime = config->vehicle.escRampTimeSecondGear;
      break;
    case 3:
      escRampTime = config->vehicle.escRampTimeThirdGear;
      break;
    default:
      escRampTime = 20;
  }

  if (millis() - escMillis > escRampTime) { // About very 20 - 75ms
    escMillis = millis();

    if (state.isFailsafe()) {
      brakeRampRate = config->vehicle.escBrakeSteps;
      driveRampRate = config->vehicle.escBrakeSteps;
    } else {
      // calulate throttle dependent brake & acceleration steps
      brakeRampRate = map(state.getThrottle(), 0, 500, 1, config->vehicle.escBrakeSteps);
      driveRampRate = map(state.getThrottle(), 0, 500, 1, config->vehicle.escAccelerationSteps);
    }

    uint16_t throttleRaw = state.getThrottleRaw();

    if (throttleRaw > config->escConfig.neutral) {
      pulse = 1; // 1 = Forward
    } else if (throttleRaw < config->escConfig.neutral) {
      pulse = -1; // -1 = Backwards
    } else {
      pulse = 0; // 0 = Neutral
    }

    if (escPulseWidth > config->escConfig.neutral) {
      escPulse = 1; // 1 = Forward
    } else if (escPulseWidth < config->escConfig.neutral) {
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

        if (pulse == 1 && state.getEngineState() == RUNNING && state.isClutch()) {
          state.setDriveState(DRIVING_FORWARD);
        } else if (pulse == -1 && state.getEngineState() == RUNNING && state.isClutch()) {
          state.setDriveState(DRIVING_BACKWARD);
        }
        break;
      case DRIVING_FORWARD:
        escController.setBraking(false);
        escController.setDriving(true);
        escController.setInReverse(false);

        // If clutch is disengaged coast to neutral
        if (!state.isClutch()) {
          if (escPulseWidth > config->escConfig.neutral) {
            escPulseWidth -= 2; // Slow coast down (slower than braking)
          } else {
            escPulseWidth = config->escConfig.neutral;
          }
        }
        // Normal driving with clutch engaged
        else if (escPulseWidth < throttleRaw && state.getSpeed() < speedLimit) {
          if (escPulseWidth > config->escConfig.neutral) {
            escPulseWidth += (driveRampRate * driveRampGain); // Faster
          } else if (escPulseWidth == config->escConfig.neutral) {
            // Apply initial takeoff punch for forward start
            int32_t target = config->escConfig.neutral + config->escConfig.fwdStartGap;
            escPulseWidth = target > config->escConfig.max ? config->escConfig.max : target;
          } else {
            escPulseWidth = config->escConfig.neutral; // Initial boost from reverse/neutral
          }
        } else if (escPulseWidth > throttleRaw && escPulseWidth > config->escConfig.neutral) {
          escPulseWidth -= (driveRampRate * driveRampGain); // Slower
        }

        if (state.getGearShift() == UP && config->vehicle.shiftingAutoThrottle) {
          // lowering RPM, if shifting up transmission
          escPulseWidth -= state.getSpeed() / 4; // Synchronize engine speed
          escPulseWidth = constrain(escPulseWidth, config->escConfig.neutral, config->escConfig.max);
        } else if (state.getGearShift() == DOWN && config->vehicle.shiftingAutoThrottle) {
          // increasing RPM, if shifting down transmission
          escPulseWidth += 50; // Synchronize engine speed
          escPulseWidth = constrain(escPulseWidth, config->escConfig.neutral, config->escConfig.max);
        }

        if (pulse == -1 && escPulse == 1) {
          state.setDriveState(BRAKING_FORWARD);
        } else if (pulse == -1 && escPulse == 0 && state.isClutch()) {
          state.setDriveState(DRIVING_BACKWARD);
        } else if (escPulse == 0 && (pulse == 0 || !state.isClutch())) {
          state.setDriveState(STANDING); // Go to standing when stopped and (throttle neutral or clutch off)
        }
        break;

      case BRAKING_FORWARD:
        escController.setBraking(true);
        escController.setDriving(false);
        escController.setInReverse(false);

        if (escPulseWidth > config->escConfig.neutral) {
          escPulseWidth -= brakeRampRate; // brake with variable deceleration
        } else if (escPulseWidth < config->escConfig.neutral && pulse == 0) {
          escPulseWidth = config->escConfig.neutral; // Overflow prevention!
        }

        if (pulse == 0 && escPulse == 1 && state.isClutch()) {
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

        // If clutch is disengaged  coast to neutral
        if (!state.isClutch()) {
          if (escPulseWidth < config->escConfig.neutral) {
            escPulseWidth += 2; // Slow coast down (slower than braking)
          } else {
            escPulseWidth = config->escConfig.neutral;
          }
        }
        // Normal driving with clutch engaged
        else if (escPulseWidth > throttleRaw && state.getSpeed() < speedLimit) {
          if (escPulseWidth < config->escConfig.neutral) {
            escPulseWidth -= driveRampRate * driveRampGain; // Faster
          } else if (escPulseWidth == config->escConfig.neutral) {
            // Apply initial takeoff punch for reverse start
            int32_t target = config->escConfig.neutral - config->escConfig.revStartGap;
            escPulseWidth = target < config->escConfig.min ? config->escConfig.min : target;
          } else {
            escPulseWidth = config->escConfig.neutral; // Initial boost from forward/neutral
          }
        } else if (escPulseWidth < throttleRaw && escPulseWidth < config->escConfig.neutral) {
          escPulseWidth += driveRampRate * driveRampGain; // Slower
        }

        if (state.getGearShift() == UP && config->vehicle.shiftingAutoThrottle) {
          // lowering RPM, if shifting up transmission
          escPulseWidth += state.getSpeed() / 4; // Synchronize engine speed
          escPulseWidth = constrain(escPulseWidth, config->escConfig.min, config->escConfig.neutral);
        } else if (state.getGearShift() == DOWN && config->vehicle.shiftingAutoThrottle) {
          // increasing RPM, if shifting down transmission
          escPulseWidth -= 50; // Synchronize engine speed
          escPulseWidth = constrain(escPulseWidth, config->escConfig.min, config->escConfig.neutral);
        }

        if (pulse == 1 && escPulse == -1) {
          state.setDriveState(BRAKING_BACKWARD);
        } else if (pulse == 1 && escPulse == 0 && state.isClutch()) {
          state.setDriveState(DRIVING_FORWARD); // Prevents state machine from hanging!
        } else if (escPulse == 0 && (pulse == 0 || !state.isClutch())) {
          state.setDriveState(STANDING); // Go to standing when stopped and (throttle neutral or clutch off)
        }
        break;

      case BRAKING_BACKWARD:
        escController.setBraking(true);
        escController.setDriving(false);
        escController.setInReverse(true);

        if (escPulseWidth < config->escConfig.neutral) {
          escPulseWidth += brakeRampRate; // brake with variable deceleration
        } else if (escPulseWidth > config->escConfig.neutral) {
          escPulseWidth = config->escConfig.neutral; // Overflow prevention!
        }

        if (pulse == 0 && escPulse == -1 && state.isClutch()) {
          state.setDriveState(DRIVING_BACKWARD);
          state.setAirBrake(true);
        } else if (pulse == 0 && escPulse == 0) {
          state.setDriveState(STANDING);
          state.setAirBrake(true);
        }
        break;
    }

    // Gain for drive ramp rate, depending on clutchEngagingPoint
    if (state.getSpeed() < config->vehicle.clutchEngagingPoint) {
      driveRampGain = 2; // prevent clutch from slipping too much (2)
    } else {
      driveRampGain = 1;
    }

    // ESC control
    escSignal = map(escPulseWidth, config->escConfig.min, config->escConfig.max, 1000, 2000);
    escController.set_esc_us(escSignal);

    // Calculate a speed value from the pulsewidth signal (used as base for engine sound RPM while
    // clutch is engaged)
    if (escPulseWidth > config->escConfig.neutral) {
      state.setSpeed(map(escPulseWidth, config->escConfig.neutral, config->escConfig.max, MIN_SPEED, MAX_SPEED));
    } else if (escPulseWidth < config->escConfig.neutral) {
      state.setSpeed(map(escPulseWidth, config->escConfig.neutral, config->escConfig.min, MIN_SPEED, MAX_SPEED));
    } else {
      state.setSpeed(0);
    }
  }
}

void initNVS() {
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    err = nvs_flash_init();
  }
  ESP_ERROR_CHECK(err);
  ESP_LOGI(TAG, "NVS initialized");
}

void mountSPIFFS() {
  esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs", .partition_label = "spiffs", .max_files = 5, .format_if_mount_failed = false};

  esp_err_t ret = esp_vfs_spiffs_register(&conf);
  if (ret != ESP_OK) {
    if (ret == ESP_ERR_INVALID_STATE) {
      ESP_LOGW(TAG, "SPIFFS already mounted");
    } else {
      ESP_LOGE(TAG, "Failed to mount SPIFFS (%s)", esp_err_to_name(ret));
      return;
    }
  }
}