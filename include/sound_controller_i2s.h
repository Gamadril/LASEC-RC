#pragma once

#include "board_config.h"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "model_state.h"
#include "vehicle.h"

extern long map(long x, long in_min, long in_max, long out_min, long out_max);
extern unsigned long micros();
extern unsigned long millis();

#define AUDIO_RATE 22050
#define DEFAULT_TICKS (4000000 / AUDIO_RATE) // timer ticks per sample at 4MHz
#define TIMER_RESOLUTION_HZ 4000000          // 4 MHz → 0.25 µs per tick

// I2S configuration - larger DMA buffers for queue-based approach
#define I2S_DMA_FRAME_COUNT 256
#define I2S_DMA_DESC_COUNT 8
#define AUDIO_QUEUE_LENGTH 32 // Queue depth for ISR->task communication

// Audio sample structure for queue
struct AudioSample {
  int16_t left;
  int16_t right;
};

// I2S globals
i2s_chan_handle_t i2s_tx_chan = nullptr;
QueueHandle_t audioQueue = nullptr;
TaskHandle_t i2sWriteTaskHandle = nullptr;

// I2S write task - runs continuously, pulls from queue and writes to I2S
void i2sWriteTask(void *parameter) {
  AudioSample sample;
  int16_t stereoSample[2];
  size_t bytesWritten;

  ESP_LOGI("I2S", "I2S write task started on core %d", xPortGetCoreID());

  while (true) {
    // Block waiting for sample from ISR (no timeout = infinite wait)
    if (xQueueReceive(audioQueue, &sample, portMAX_DELAY) == pdTRUE) {
      stereoSample[0] = sample.left;
      stereoSample[1] = sample.right;

      // Blocking write to I2S - DMA pacing handles timing
      esp_err_t ret = i2s_channel_write(i2s_tx_chan, stereoSample, sizeof(stereoSample),
                                        &bytesWritten, portMAX_DELAY);
      if (ret != ESP_OK || bytesWritten != sizeof(stereoSample)) {
        // Only log errors occasionally to avoid spam
        static uint32_t errCount = 0;
        if (++errCount % 1000 == 0) {
          ESP_LOGW("I2S", "Write error or incomplete: %d, wrote %d/%d bytes", ret, bytesWritten,
                   sizeof(stereoSample));
        }
      }
    }
  }
}

bool fixed_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
                    void *user_ctx);
bool variable_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
                       void *user_ctx);

uint32_t maxSampleInterval;
uint32_t minSampleInterval;
// Interrupt timer for variable sample rate playback (engine sound)
gptimer_handle_t variableTimer = NULL;
volatile uint32_t variableTimerTicks = DEFAULT_TICKS;
// Interrupt timer for fixed sample rate playback (horn etc., playing in parallel with engine sound)
gptimer_handle_t fixedTimer = NULL;
volatile uint16_t engineSampleRate = 0; // Engine sample rate

// Track last applied alarm counts to avoid redundant reconfiguration
volatile uint32_t appliedVariableTimerTicks = DEFAULT_TICKS;

// Deferred timer update for ISR safety
volatile bool pendingTimerUpdate = false;
volatile uint32_t newTimerTicks = 0;

class SoundController;
extern SoundController soundController;

class SoundController {
public:
  void reinit() {
    minSampleInterval = DEFAULT_TICKS * 100 / vehicle->maxRpmPercentage;
  }

  void onTurnSignal(bool value) {
    turnSignal = value;
  }

  void onHorn(bool value) {
    horn = value;
    if (horn) {
      hornLatch = true;
    }
  }

  void onIgnition(bool value) {
    ignition = value;
  }

  void onCoupling() {
    coupling = true;
  }

  void onUncoupling() {
    uncoupling = true;
  }

  void onRpmChange() {
    uint16_t value = soundController.modelState->getRPM();
    engineSampleRate = map(value, MIN_RPM, MAX_RPM, maxSampleInterval, minSampleInterval); // Idle

    // Calculate RPM dependent Diesel knock volume
    if (vehicle->dieselKnockDependsOnRPM) {
      if (value > 400) {
        rpmDependentKnockVolume = map(value, 400, MAX_RPM, 5, 100);
      } else {
        rpmDependentKnockVolume = 5;
      }
    }

    // Calculate engine rpm dependent jake brake volume
    if (vehicle->jakeBrakeEnabled) {
      if (soundController.modelState->isEngineRunning()) {
        rpmDependentJakeBrakeVolume =
            map(value, MIN_RPM, MAX_RPM, vehicle->jakeBrakeIdleVolume, 100);
      } else {
        rpmDependentJakeBrakeVolume = vehicle->jakeBrakeIdleVolume;
      }
    }

    // Calculate engine rpm dependent turbo volume
    if (vehicle->turboEnabled) {
      if (soundController.modelState->isEngineRunning()) {
        throttleDependentTurboVolume = map(value, MIN_RPM, MAX_RPM, vehicle->turboIdleVolume, 100);
      } else {
        throttleDependentTurboVolume = vehicle->turboIdleVolume;
      }
    }

    // Calculate engine rpm dependent cooling fan volume
    if (vehicle->fanEnabled) {
      if (soundController.modelState->isEngineRunning() && (value > vehicle->fanStartPoint)) {
        throttleDependentFanVolume =
            map(value, vehicle->fanStartPoint, MAX_RPM, vehicle->fanIdleVolume, 100);
      } else {
        throttleDependentFanVolume = vehicle->fanIdleVolume;
      }
    }

    // Calculate throttle dependent supercharger volume
    if (vehicle->chargerEnabled) {
      if (soundController.modelState->isEngineRunningNotBreaking() &&
          (value > vehicle->chargerStartPoint))
        throttleDependentChargerVolume = map(currentThrottleFaded, vehicle->chargerStartPoint,
                                             MAX_RPM, vehicle->chargerIdleVolume, 100);
      else
        throttleDependentChargerVolume = vehicle->chargerIdleVolume;
    }

    // Calculate engine rpm dependent wastegate volume
    if (vehicle->wastegateEnabled) {
      if (soundController.modelState->isEngineRunning()) {
        rpmDependentWastegateVolume =
            map(value, MIN_RPM, MAX_RPM, vehicle->wastegateIdleVolume, 100);
      } else {
        rpmDependentWastegateVolume = vehicle->wastegateIdleVolume;
      }
    }
  }

  void onThrottleChange(uint16_t value) {
    static unsigned long throttleFaderMicros = 0;
    static uint16_t lastThrottle = 0;

    if (vehicle->wastegateEnabled) {
      // Prevent Wastegate from being triggered while downshifting
      if (soundController.modelState->getGearShift() == DOWN) {
        wastegateMillis = millis();
      }

      // Trigger Wastegate, if throttle rapidly dropped
      if (lastThrottle - value > 70 && !modelState->isBraking() &&
          millis() - wastegateMillis > 1000) {
        wastegateMillis = millis();
        wastegate = true;
      }
    }

    if (micros() - throttleFaderMicros > 500) { // Every 0.5ms
      throttleFaderMicros = micros();

      if (currentThrottleFaded < value && currentThrottleFaded < 499) {
        currentThrottleFaded += 32; // Very fast fade-in
        if (currentThrottleFaded > value)
          currentThrottleFaded = value;
      } else if (currentThrottleFaded > value && currentThrottleFaded > 32) {
        currentThrottleFaded -= 32; // Very fast fade-out
        if (currentThrottleFaded < value)
          currentThrottleFaded = value;
      }

      // Calculate throttle dependent engine idle volume
      if (soundController.modelState->isEngineRunningNotBreaking()) {
        throttleDependentVolume = map(currentThrottleFaded, 0, 500, vehicle->engineIdleVolume,
                                      vehicle->fullThrottleVolume);
      } else {
        throttleDependentVolume = vehicle->engineIdleVolume;
      }

      // Calculate throttle dependent engine rev volume
      if (vehicle->revSoundEnabled) {
        if (soundController.modelState->isEngineRunningNotBreaking()) {
          throttleDependentRevVolume = map(currentThrottleFaded, 0, 500, vehicle->engineRevVolume,
                                           vehicle->fullThrottleVolume);
        } else {
          throttleDependentRevVolume = vehicle->engineRevVolume;
        }
      }

      // Calculate throttle dependent Diesel knock volume
      if (soundController.modelState->isEngineRunningNotBreaking() &&
          (currentThrottleFaded > vehicle->dieselKnockStartPoint)) {
        throttleDependentKnockVolume = map(currentThrottleFaded, vehicle->dieselKnockStartPoint,
                                           500, vehicle->dieselKnockIdleVolume, 100);
      } else {
        throttleDependentKnockVolume = vehicle->dieselKnockIdleVolume;
      }
    }
    lastThrottle = value;
  }

  void init(ModelState *modelState, Vehicle *vehicle, Config *config) {
    this->modelState = modelState;
    this->vehicle = vehicle;
    this->config = config;

    /* I2S init */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t err = i2s_new_channel(&chan_cfg, &i2s_tx_chan, nullptr);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "i2s channel create failed: %d", err);
      return;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(AUDIO_RATE),
        .slot_cfg =
            I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = static_cast<gpio_num_t>(PIN_I2S_BCLK),
                .ws = static_cast<gpio_num_t>(PIN_I2S_LRC),
                .dout = static_cast<gpio_num_t>(PIN_I2S_DOUT),
                .din = I2S_GPIO_UNUSED,
                .invert_flags =
                    {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
            },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    err = i2s_channel_init_std_mode(i2s_tx_chan, &std_cfg);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "i2s std mode init failed: %d", err);
      return;
    }

    err = i2s_channel_enable(i2s_tx_chan);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "i2s channel enable failed: %d", err);
      return;
    }

    // Create audio queue for ISR->task communication
    audioQueue = xQueueCreate(AUDIO_QUEUE_LENGTH, sizeof(AudioSample));
    if (audioQueue == nullptr) {
      ESP_LOGE("SND", "Failed to create audio queue");
      return;
    }

    // Create high-priority I2S write task pinned to core 1 (app core)
    BaseType_t taskCreated =
        xTaskCreatePinnedToCore(i2sWriteTask,             // Task function
                                "I2S_Write",              // Task name
                                4096,                     // Stack size
                                nullptr,                  // Parameters
                                configMAX_PRIORITIES - 1, // High priority (just below ISR)
                                &i2sWriteTaskHandle,      // Task handle
                                1 // Core 1 (app core, core 0 runs protocol stack)
        );

    if (taskCreated != pdPASS) {
      ESP_LOGE("SND", "Failed to create I2S write task");
      return;
    }

    // Send a few silent samples to initialize the MAX98357
    AudioSample silence = {0, 0};
    for (int i = 0; i < 100; i++) {
      xQueueSend(audioQueue, &silence, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(1));
    }

    minSampleInterval = DEFAULT_TICKS * 100 / vehicle->maxRpmPercentage;
    maxSampleInterval = DEFAULT_TICKS;
    variableTimerTicks = maxSampleInterval;
    appliedVariableTimerTicks = variableTimerTicks;

    // Variable timer removed - all audio now handled by fixed timer with phase accumulators

    // Interrupt timer for fixed sample rate playback
    gptimer_config_t timer_config_fixed = {};
    timer_config_fixed.clk_src = GPTIMER_CLK_SRC_DEFAULT;   // Select the default clock source
    timer_config_fixed.direction = GPTIMER_COUNT_UP;        // Counting direction is up
    timer_config_fixed.resolution_hz = TIMER_RESOLUTION_HZ; // Resolution is 4 MHz, 1 tick = 0.25 µs

    // Create a timer instance
    err = gptimer_new_timer(&timer_config_fixed, &fixedTimer);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "gptimer fixed new failed: %d", err);
      return;
    }

    // Configure alarm (like timerAlarmWrite)
    gptimer_alarm_config_t alarm_config_fixed = {};
    alarm_config_fixed.alarm_count = DEFAULT_TICKS;       // already in timer ticks
    alarm_config_fixed.reload_count = 0;                  // start from 0
    alarm_config_fixed.flags.auto_reload_on_alarm = true; // periodic mode
    err = gptimer_set_alarm_action(fixedTimer, &alarm_config_fixed);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "gptimer fixed set_alarm failed: %d", err);
      return;
    }

    // Attach ISR (like timerAttachInterrupt)
    gptimer_event_callbacks_t cbs_fixed = {
        .on_alarm = fixed_timer_cb,
    };
    err = gptimer_register_event_callbacks(fixedTimer, &cbs_fixed, NULL);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "gptimer fixed cb reg failed: %d", err);
      return;
    }

    // Enable + start timer (like timerAlarmEnable)
    err = gptimer_enable(fixedTimer);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "gptimer fixed enable failed: %d", err);
      return;
    }
    err = gptimer_start(fixedTimer);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "gptimer fixed start failed: %d", err);
      return;
    }
  }

  void deinit() {
    // No I2S task to stop

    // I2S amplifier is always enabled (no control needed)

    // Clean up I2S channel
    if (i2s_tx_chan) {
      i2s_channel_disable(i2s_tx_chan);
      i2s_del_channel(i2s_tx_chan);
      i2s_tx_chan = nullptr;
    }

    if (variableTimer) {
      gptimer_stop(variableTimer);
      gptimer_disable(variableTimer);
      gptimer_del_timer(variableTimer);
      variableTimer = nullptr;
    }
    if (fixedTimer) {
      gptimer_stop(fixedTimer);
      gptimer_disable(fixedTimer);
      gptimer_del_timer(fixedTimer);
      fixedTimer = nullptr;
    }

    this->modelState = NULL;
    this->vehicle = NULL;
    this->config = NULL;
  }

  ModelState *modelState = NULL;
  Vehicle *vehicle = NULL;
  Config *config = NULL;

  uint16_t currentThrottleFaded = 0;
  uint16_t throttleDependentVolume = 0;
  uint16_t throttleDependentTurboVolume = 0;
  uint16_t throttleDependentFanVolume = 0;
  uint16_t throttleDependentChargerVolume = 0;
  uint16_t throttleDependentRevVolume = 0;
  uint16_t throttleDependentKnockVolume = 0;
  uint16_t rpmDependentKnockVolume = 0;
  uint16_t rpmDependentJakeBrakeVolume = 0;
  uint16_t rpmDependentWastegateVolume = 0;

  uint32_t wastegateMillis = 0;

  // Active, if engine is jake braking
  bool engineJakeBraking = false;
  // Trigger Diesel ignition "knock"
  bool dieselKnockTrigger = false;
  // The first  Diesel ignition "knock" per sequence
  bool dieselKnockTriggerFirst = false;

  bool turnSignal = false;
  bool ignition = false;
  bool horn = false;
  bool hornLatch = false;
  bool wastegate = false;
  bool coupling = false;
  bool uncoupling = false;
  bool shifting = false;
};

// Timer update function removed - no longer needed with single fixed-rate timer

bool IRAM_ATTR fixed_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
                              void *user_ctx) {
  static uint32_t curHornSample = 0;         // Index of currently loaded horn sample
  static uint32_t curReversingSample = 0;    // Index of currently loaded reversing beep sample
  static uint32_t curIndicatorSample = 0;    // Index of currently loaded indicator tick sample
  static uint32_t curWastegateSample = 0;    // Index of currently loaded wastegate sample
  static uint32_t curBrakeSample = 0;        // Index of currently loaded brake sound sample
  static uint32_t curParkingBrakeSample = 0; // Index of currently loaded brake sound sample
  static uint32_t curShiftingSample = 0;     // Index of currently loaded shifting sample
  static uint32_t curCouplingSample = 0;     // Index of currently loaded trailer coupling sample
  static uint32_t curUncouplingSample = 0;   // Index of currently loaded trailer uncoupling sample
  static uint32_t curDieselKnockSample = 0;  // Index of currently loaded Diesel knock sample

  static int32_t a1 = 0; // Group "a" mixer signal (horn)
  static int32_t b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0, b7 = 0, b8 = 0, b9 = 0,
                 b10 = 0; // Group "b" mixer signals

  // Diesel knock state
  static uint32_t lastKnockPhase = 0;
  static bool knockSilent = false;
  static uint8_t curKnockCylinder = 0;

  // Local channel values for this ISR invocation
  int16_t leftChannelValue = 0;
  int16_t rightChannelValue = 0;

  // Group "a" (horn)
  if (soundController.horn || soundController.hornLatch) {
    if (curHornSample < sizeof(hornSamples) - 1) {
      a1 = (hornSamples[curHornSample] * soundController.vehicle->hornVolume / 100);
      curHornSample++;
      if (soundController.horn && curHornSample == hornLoopEnd) {
        curHornSample = hornLoopBegin; // Loop, if trigger still present
      }
    } else {
      curHornSample = 0;
      a1 = 0;
      soundController.hornLatch = false;
    }
  }

  // Group "b" (other sounds)

  // Reversing beep sound "b1"
  if (soundController.modelState->isMovingBackward()) {
    if (curReversingSample < sizeof(reversingSamples) - 1) {
      b1 = (reversingSamples[curReversingSample] * soundController.vehicle->reversingVolume / 100);
      curReversingSample++;
    } else {
      curReversingSample = 0;
    }
  } else {
    curReversingSample = 0; // ensure, next sound will start @ first sample
    b1 = 0;
  }

  // Indicator tick sound "b2"
  if (soundController.turnSignal) {
    if (curIndicatorSample < sizeof(mb_indicatorSamples) - 1) {
      b2 = (mb_indicatorSamples[curIndicatorSample] * soundController.vehicle->indicatorVolume /
            100);
      curIndicatorSample++;
    } else {
      soundController.turnSignal = false;
    }
  } else {
    curIndicatorSample = 0; // ensure, next sound will start @ first sample
    b2 = 0;
  }

  // Wastegate (blowoff) sound, triggered after rapid throttle drop "b3"
  if (soundController.vehicle->wastegateEnabled) {
    if (soundController.wastegate) {
      if (curWastegateSample < sizeof(wastegateSamples) - 1) {
        b3 = (wastegateSamples[curWastegateSample] * soundController.rpmDependentWastegateVolume /
              100 * soundController.vehicle->wastegateVolume / 100);
        curWastegateSample++;
      } else {
        soundController.wastegate = false;
      }
    } else {
      b3 = 0;
      curWastegateSample = 0; // ensure, next sound will start @ first sample
    }
  }

  // Air brake release sound "b4", triggered after stop
  if (soundController.modelState->isAirBrake()) {
    if (curBrakeSample < sizeof(brakeSamples) - 1) {
      b4 = (brakeSamples[curBrakeSample] * soundController.vehicle->brakeVolume / 100);
      curBrakeSample++;
    } else {
      soundController.modelState->setAirBrake(false);
    }
  } else {
    b4 = 0;
    curBrakeSample = 0; // ensure, next sound will start @ first sample
  }

  // Air parking brake attaching sound "b5", triggered after engine off
  if (soundController.modelState->isParkingBrake()) {
    if (curParkingBrakeSample < sizeof(parkingBrakeSamples) - 1) {
      b5 = (parkingBrakeSamples[curParkingBrakeSample] *
            soundController.vehicle->parkingBrakeVolume / 100);
      curParkingBrakeSample++;
    } else {
      soundController.modelState->setParkingBrake(false);
    }
  } else {
    b5 = 0;
    curParkingBrakeSample = 0; // ensure, next sound will start @ first sample
  }

  // Pneumatic gear shifting sound "b6", triggered while shifting the TAMIYA 3 speed transmission
  if (soundController.shifting && soundController.modelState->isEngineRunning()) {
    if (curShiftingSample < sizeof(shiftingSamples) - 1) {
      b6 = (shiftingSamples[curShiftingSample] * soundController.vehicle->shiftingVolume / 100);
      curShiftingSample++;
    } else {
      soundController.shifting = false;
    }
  } else {
    b6 = 0;
    curShiftingSample = 0; // ensure, next sound will start @ first sample
  }

  // Diesel ignition "knock" is played in fixed sample rate section, because we don't want changing
  // pitch!
  if (soundController.dieselKnockTriggerFirst) {
    soundController.dieselKnockTriggerFirst = false;
    curKnockCylinder = 0;
  }

  if (soundController.dieselKnockTrigger) {
    soundController.dieselKnockTrigger = false;
    curKnockCylinder++; // Count ignition sequence
    curDieselKnockSample = 0;
  }

  knockSilent = true;
  for (uint8_t i = 0; i < sizeof(soundController.vehicle->dieselKnockCylinders); i++) {
    if (soundController.vehicle->dieselKnockCylinders[i] == curKnockCylinder) {
      knockSilent = false;
      break;
    }
  }

  /*
  if (curDieselKnockSample < sizeof(knockSamples)) {
    b7 = knockSamples[curDieselKnockSample] * soundController.vehicle->dieselKnockVolume / 100 *
         soundController.throttleDependentKnockVolume / 100;

    if (soundController.vehicle->dieselKnockDependsOnRPM) {
      b7 = b7 * soundController.rpmDependentKnockVolume / 100;
    }

    curDieselKnockSample++;
    if (knockSilent) {
      // changing knock volume according to engine type and cylinder!
      b7 = b7 * soundController.vehicle->dieselKnockAdaptiveVolume / 100;
    }
  }
  */
  if (soundController.vehicle->couplingSoundEnabled) {
    // Trailer coupling sound "b8", triggered by switch
    if (soundController.coupling) {
      if (curCouplingSample < sizeof(couplingSamples) - 1) {
        b8 = (couplingSamples[curCouplingSample] * soundController.vehicle->couplingVolume / 100);
        curCouplingSample++;
      } else {
        soundController.coupling = false;
      }
    } else {
      b8 = 0;
      curCouplingSample = 0; // ensure, next sound will start @ first sample
    }

    // Trailer uncoupling sound "b9", triggered by switch
    if (soundController.uncoupling) {
      if (curUncouplingSample < sizeof(uncouplingSamples) - 1) {
        b9 = (uncouplingSamples[curUncouplingSample] * soundController.vehicle->couplingVolume /
              100);
        curUncouplingSample++;
      } else {
        soundController.uncoupling = false;
      }
    } else {
      b9 = 0;
      curUncouplingSample = 0; // ensure, next sound will start @ first sample
    }
  }

  // Diesel knock sound - triggered by phase accumulator in engine resampler
  if (soundController.dieselKnockTrigger) {
    if (curDieselKnockSample < sizeof(knockSamples) - 1) {
      if (soundController.dieselKnockTriggerFirst) {
        b10 = (knockSamples[curDieselKnockSample] * soundController.throttleDependentKnockVolume /
               100 * soundController.vehicle->dieselKnockVolume / 100);
      } else {
        if (knockSilent) {
          b10 = (knockSamples[curDieselKnockSample] * soundController.throttleDependentKnockVolume /
                 100 * soundController.vehicle->dieselKnockVolume / 100 / 3);
        } else {
          b10 = (knockSamples[curDieselKnockSample] * soundController.throttleDependentKnockVolume /
                 100 * soundController.vehicle->dieselKnockVolume / 100);
        }
      }
      curDieselKnockSample++;
    } else {
      soundController.dieselKnockTrigger = false;
      curDieselKnockSample = 0;
      curKnockCylinder++;
      // dieselKnockCylinders is an array, check if current cylinder value is non-zero
      if (curKnockCylinder >= 4 ||
          soundController.vehicle->dieselKnockCylinders[curKnockCylinder] == 0) {
        curKnockCylinder = 0;
      }
      knockSilent = !knockSilent;
    }
  } else {
    b10 = 0;
  }

  // Resample engine at fixed I2S rate using 16.16 phase accumulators
  static uint32_t phaseIdle = 0;
  static uint32_t phaseRev = 0;
  static uint32_t phaseJake = 0;
  static uint32_t phaseStart = 0;
  static uint32_t phaseTurbo = 0;
  static uint32_t phaseFan = 0;
  static uint32_t phaseCharger = 0;
  static uint32_t attenuatorMillis = 0;
  static uint16_t attenuator = 1;
  static uint16_t speedPercentage = 100;
  static EngineState prevEs = OFF;
  EngineState es = soundController.modelState->getEngineState();
  if (es != prevEs) {
    // Reset phases on state transition for clean starts
    if (es == STARTING) {
      phaseStart = 0;
    }
    if (es == RUNNING) {
      phaseIdle = phaseRev = phaseJake = 0;
      lastKnockPhase = 0;
      soundController.dieselKnockTrigger = true;
      soundController.dieselKnockTriggerFirst = true;
    }
    if (es == STOPPING) {
      speedPercentage = 100;
      attenuator = 1;
      attenuatorMillis = millis();
    }
    prevEs = es;
  }

  if (es == STARTING) {
    // Play start sample at default ticks like original
    uint32_t ticks = DEFAULT_TICKS;
    uint32_t phaseInc = (uint32_t)(((uint64_t)TIMER_RESOLUTION_HZ << 16) /
                                   ((uint64_t)ticks * (uint64_t)AUDIO_RATE));
    phaseStart += phaseInc;
    uint32_t lenStart = (uint32_t)sizeof(startSamples);
    int32_t startInterp = 0;
    if (lenStart > 1) {
      uint32_t i0 = (phaseStart >> 16) % lenStart;

      // Check if we've reached the end of start sample - transition to RUNNING
      if (i0 >= lenStart - 1) {
        soundController.modelState->setEngineState(RUNNING);
        soundController.modelState->setAirBrake(true);
        leftChannelValue = 0;
      } else {
        uint32_t j0 = (i0 + 1) % lenStart;
        uint32_t frac0 = (phaseStart & 0xFFFF);
        int16_t t0 = (int16_t)((int8_t)startSamples[i0]) << 8;
        int16_t t1 = (int16_t)((int8_t)startSamples[j0]) << 8;
        startInterp = t0 + ((int32_t)(t1 - t0) * (int32_t)frac0 >> 16);

        int32_t startScaled = startInterp;
        startScaled = (startScaled * soundController.throttleDependentVolume) / 100;
        startScaled = (startScaled * soundController.vehicle->startVolume) / 100;
        startScaled = (startScaled * soundController.config->volume) / 100;
        leftChannelValue = (int16_t)constrain(startScaled, -32768, 32767);
      }
    }
  } else if (es == RUNNING) {
    // Use freshest RPM-derived ticks to minimize latency
    uint32_t ticks = engineSampleRate ? engineSampleRate : variableTimerTicks;
    if (ticks == 0) {
      ticks = DEFAULT_TICKS;
    }
    uint32_t phaseInc = (uint32_t)(((uint64_t)TIMER_RESOLUTION_HZ << 16) /
                                   ((uint64_t)ticks * (uint64_t)AUDIO_RATE));

    int32_t engineMixed = 0;

    if (!soundController.modelState->isJakeBrake()) {
      // Idle
      phaseIdle += phaseInc;
      uint32_t lenIdle = (uint32_t)sizeof(samples);
      // Wrap phase to prevent overflow
      uint32_t maxPhaseIdle = lenIdle << 16;
      if (phaseIdle >= maxPhaseIdle)
        phaseIdle -= maxPhaseIdle;

      int32_t idleInterp = 0;
      if (lenIdle > 1) {
        uint32_t i = (phaseIdle >> 16) % lenIdle;

        // Check if we wrapped around - trigger jake brake if needed
        if (i == 0 && soundController.modelState->isJakeBrake()) {
          soundController.engineJakeBraking = true;
        }

        uint32_t j = (i + 1) % lenIdle;
        uint32_t frac = (phaseIdle & 0xFFFF);
        int16_t s0 = (int16_t)((int8_t)samples[i]) << 8;
        int16_t s1 = (int16_t)((int8_t)samples[j]) << 8;
        idleInterp = s0 + ((int32_t)(s1 - s0) * (int32_t)frac >> 16);

        // Diesel knock triggering based on phase position
        uint32_t knockInterval = (lenIdle << 16) / soundController.vehicle->dieselKnockInterval;
        uint32_t phaseDiff = (phaseIdle >= lastKnockPhase)
                                 ? (phaseIdle - lastKnockPhase)
                                 : (maxPhaseIdle - lastKnockPhase + phaseIdle);
        if (phaseDiff >= knockInterval) {
          soundController.dieselKnockTrigger = true;
          soundController.dieselKnockTriggerFirst = (lastKnockPhase == 0);
          lastKnockPhase = phaseIdle;
        }
      }

      // Rev (optional)
      int32_t revInterp = 0;
      if (soundController.vehicle->revSoundEnabled) {
        phaseRev += phaseInc;
        uint32_t lenRev = (uint32_t)sizeof(revSamples);
        // Wrap phase to prevent overflow
        uint32_t maxPhaseRev = lenRev << 16;
        if (phaseRev >= maxPhaseRev)
          phaseRev -= maxPhaseRev;

        if (lenRev > 1) {
          uint32_t i2 = (phaseRev >> 16) % lenRev;
          uint32_t j2 = (i2 + 1) % lenRev;
          uint32_t frac2 = (phaseRev & 0xFFFF);
          int16_t r0 = (int16_t)((int8_t)revSamples[i2]) << 8;
          int16_t r1 = (int16_t)((int8_t)revSamples[j2]) << 8;
          revInterp = r0 + ((int32_t)(r1 - r0) * (int32_t)frac2 >> 16);
        }
      }

      // RPM-based mix like Arduino path
      uint16_t rpm = soundController.modelState->getRPM();
      uint8_t a1Multi;
      if (rpm > soundController.vehicle->revSwitchPoint) {
        a1Multi =
            map(rpm, soundController.vehicle->idleEndPoint, soundController.vehicle->revSwitchPoint,
                0, soundController.vehicle->idleVolumeProportion);
      } else {
        a1Multi = soundController.vehicle->idleVolumeProportion;
      }
      if (rpm > soundController.vehicle->idleEndPoint) {
        a1Multi = 0;
      }

      int32_t idleScaled = idleInterp;
      idleScaled = (idleScaled * soundController.throttleDependentVolume) / 100;
      idleScaled = (idleScaled * soundController.vehicle->idleVolume) / 100;
      idleScaled = (idleScaled * a1Multi) / 100;

      int32_t revScaled = revInterp;
      revScaled = (revScaled * soundController.throttleDependentRevVolume) / 100;
      revScaled = (revScaled * soundController.vehicle->revVolume) / 100;
      revScaled = (revScaled * (100 - a1Multi)) / 100;

      engineMixed = idleScaled + revScaled;
    } else if (soundController.vehicle->jakeBrakeEnabled) {
      phaseJake += phaseInc;
      uint32_t lenJake = (uint32_t)sizeof(jakeBrakeSamples);
      int32_t jakeInterp = 0;
      if (lenJake > 1) {
        uint32_t i3 = (phaseJake >> 16) % lenJake;

        // Check if jake brake sample finished
        if (i3 >= lenJake - 1) {
          phaseJake = 0;
          if (!soundController.modelState->isJakeBrake()) {
            soundController.engineJakeBraking = false;
          }
        }

        uint32_t j3 = (i3 + 1) % lenJake;
        uint32_t frac3 = (phaseJake & 0xFFFF);
        int16_t jb0 = (int16_t)((int8_t)jakeBrakeSamples[i3]) << 8;
        int16_t jb1 = (int16_t)((int8_t)jakeBrakeSamples[j3]) << 8;
        jakeInterp = jb0 + ((int32_t)(jb1 - jb0) * (int32_t)frac3 >> 16);
      }
      // Jake volume
      engineMixed = jakeInterp;
      engineMixed = (engineMixed * soundController.rpmDependentJakeBrakeVolume) / 100;
      engineMixed = (engineMixed * soundController.vehicle->jakeBrakeVolume) / 100;

      // Reset idle/rev phases while jake braking
      phaseIdle = 0;
      phaseRev = 0;
    }

    // Turbo sound (resampled at engine rate)
    int32_t turboRaw = 0;
    if (soundController.vehicle->turboEnabled) {
      phaseTurbo += phaseInc;
      uint32_t lenTurbo = (uint32_t)sizeof(turboSamples);
      if (lenTurbo > 1) {
        // Wrap phase to prevent overflow
        uint32_t maxPhase = lenTurbo << 16;
        if (phaseTurbo >= maxPhase)
          phaseTurbo -= maxPhase;

        uint32_t i4 = (phaseTurbo >> 16) % lenTurbo;
        uint32_t j4 = (i4 + 1) % lenTurbo;
        uint32_t frac4 = (phaseTurbo & 0xFFFF);
        int16_t t0 = (int16_t)((int8_t)turboSamples[i4]) << 8;
        int16_t t1 = (int16_t)((int8_t)turboSamples[j4]) << 8;
        int32_t turboInterp = t0 + ((int32_t)(t1 - t0) * (int32_t)frac4 >> 16);
        // Convert to 8-bit equivalent for mixing
        turboRaw = (turboInterp >> 8) * soundController.throttleDependentTurboVolume / 100 *
                   soundController.vehicle->turboVolume / 100;
      }
    }

    // Fan sound (resampled at engine rate)
    int32_t fanRaw = 0;
    if (soundController.vehicle->fanEnabled) {
      phaseFan += phaseInc;
      uint32_t lenFan = (uint32_t)sizeof(fanSamples);
      if (lenFan > 1) {
        // Wrap phase to prevent overflow
        uint32_t maxPhase = lenFan << 16;
        if (phaseFan >= maxPhase)
          phaseFan -= maxPhase;

        uint32_t i5 = (phaseFan >> 16) % lenFan;
        uint32_t j5 = (i5 + 1) % lenFan;
        uint32_t frac5 = (phaseFan & 0xFFFF);
        int16_t f0 = (int16_t)((int8_t)fanSamples[i5]) << 8;
        int16_t f1 = (int16_t)((int8_t)fanSamples[j5]) << 8;
        int32_t fanInterp = f0 + ((int32_t)(f1 - f0) * (int32_t)frac5 >> 16);
        fanRaw = (fanInterp >> 8) * soundController.throttleDependentFanVolume / 100 *
                 soundController.vehicle->fanVolume / 100;
      }
    }

    // Supercharger sound (resampled at engine rate)
    int32_t chargerRaw = 0;
    if (soundController.vehicle->chargerEnabled) {
      phaseCharger += phaseInc;
      uint32_t lenCharger = (uint32_t)sizeof(chargerSamples);
      if (lenCharger > 1) {
        // Wrap phase to prevent overflow
        uint32_t maxPhase = lenCharger << 16;
        if (phaseCharger >= maxPhase)
          phaseCharger -= maxPhase;

        uint32_t i6 = (phaseCharger >> 16) % lenCharger;
        uint32_t j6 = (i6 + 1) % lenCharger;
        uint32_t frac6 = (phaseCharger & 0xFFFF);
        int16_t c0 = (int16_t)((int8_t)chargerSamples[i6]) << 8;
        int16_t c1 = (int16_t)((int8_t)chargerSamples[j6]) << 8;
        int32_t chargerInterp = c0 + ((int32_t)(c1 - c0) * (int32_t)frac6 >> 16);
        chargerRaw = (chargerInterp >> 8) * soundController.throttleDependentChargerVolume / 100 *
                     soundController.vehicle->chargerVolume / 100;
      }
    }

    // Convert engineMixed to 8-bit equivalent for mixing
    int32_t engineRaw = engineMixed >> 8;

    // Mix matching original DAC formula: (a * 8/10) + (b / 2) + (c / 5) + (d / 5) + (e / 5)
    int32_t mixed8bit = ((engineRaw * 8 / 10) + (turboRaw / 5) + (fanRaw / 5) + (chargerRaw / 5));

    // Convert engine mix to 16-bit and apply master volume for LEFT channel
    int32_t totalMixed = (mixed8bit << 8) * soundController.config->volume / 100;
    leftChannelValue = (int16_t)constrain(totalMixed, -32768, 32767);

    // Check for engine shutdown (ignition off while running)
    if (!soundController.ignition) {
      soundController.modelState->setEngineState(STOPPING);
    }
  } else if (es == STOPPING) {
    // Engine stopping - play idle sample slowing down and fading out
    uint32_t ticks = DEFAULT_TICKS * speedPercentage / 100;
    if (ticks == 0)
      ticks = DEFAULT_TICKS;
    uint32_t phaseInc = (uint32_t)(((uint64_t)TIMER_RESOLUTION_HZ << 16) /
                                   ((uint64_t)ticks * (uint64_t)AUDIO_RATE));
    phaseIdle += phaseInc;
    uint32_t lenIdle = (uint32_t)sizeof(samples);
    int32_t idleInterp = 0;
    if (lenIdle > 1) {
      uint32_t i = (phaseIdle >> 16) % lenIdle;
      uint32_t j = (i + 1) % lenIdle;
      uint32_t frac = (phaseIdle & 0xFFFF);
      int16_t s0 = (int16_t)((int8_t)samples[i]) << 8;
      int16_t s1 = (int16_t)((int8_t)samples[j]) << 8;
      idleInterp = s0 + ((int32_t)(s1 - s0) * (int32_t)frac >> 16);
    }

    // Apply fade out
    int32_t stoppingScaled = idleInterp;
    stoppingScaled = (stoppingScaled * soundController.throttleDependentVolume) / 100;
    stoppingScaled = (stoppingScaled * soundController.vehicle->idleVolume) / 100;
    stoppingScaled = (stoppingScaled * soundController.config->volume) / 100;
    stoppingScaled = stoppingScaled / attenuator;
    leftChannelValue = (int16_t)constrain(stoppingScaled, -32768, 32767);

    // Fade engine sound out every 100ms
    if (millis() - attenuatorMillis > 100) {
      attenuatorMillis = millis();
      attenuator++;
      speedPercentage += 20;
    }

    // Transition to PARKING_BRAKE when fully stopped
    if (attenuator >= 50 || speedPercentage >= 500) {
      soundController.modelState->setEngineState(PARKING_BRAKE);
      soundController.modelState->setParkingBrake(true);
      leftChannelValue = 0;
    }
  } else if (es == PARKING_BRAKE) {
    // Check if parking brake released to go back to OFF
    if (!soundController.modelState->isParkingBrake()) {
      soundController.modelState->setEngineState(OFF);
    }
    leftChannelValue = 0;
  } else {
    // Engine OFF or unknown state
    leftChannelValue = 0;
  }

  // Mix group "a" and "b" sounds (horn, reversing, indicators, etc.) for RIGHT channel
  // These sounds play regardless of engine state
  int32_t groupA = (a1 * 8 / 10);
  int32_t groupB = (b1 + b2 + b3 + b4 + b5 + b6 + b7 + b8 + b9 + b10) / 2;
  int32_t rightMixed = ((groupA + groupB) << 8) * soundController.config->volume / 100;
  rightChannelValue = (int16_t)constrain(rightMixed, -32768, 32767);

  // Push sample to queue for I2S write task (non-blocking from ISR)
  AudioSample sample = {leftChannelValue, rightChannelValue};
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // Send to queue from ISR - if queue full, drop sample (shouldn't happen with proper sizing)
  xQueueSendFromISR(audioQueue, &sample, &xHigherPriorityTaskWoken);

  // Yield to higher priority task if needed
  if (xHigherPriorityTaskWoken == pdTRUE) {
    portYIELD_FROM_ISR();
  }

  return true;
}