#pragma once

#include "board_config.h"
#include "driver/dac_oneshot.h"
#include "driver/gptimer.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "model_state.h"
#include "vehicle.h"

extern long map(long x, long in_min, long in_max, long out_min, long out_max);
extern unsigned long micros();
extern unsigned long millis();

#define AUDIO_RATE 22050
#define DEFAULT_TICKS (4000000 / AUDIO_RATE) // timer ticks per sample at 4MHz
#define TIMER_RESOLUTION_HZ 4000000          // 4 MHz → 0.25 µs per tick

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
        currentThrottleFaded += 2;
      } else if (currentThrottleFaded > value && currentThrottleFaded > 2) {
        currentThrottleFaded -= 2;
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

    /* DAC oneshot init */
    dac_oneshot_config_t chan0_cfg = {
        .chan_id = DAC_CHAN_0,
    };
    esp_err_t err = dac_oneshot_new_channel(&chan0_cfg, &this->chan0_handle);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "dac chan0 create failed: %d", err);
      return;
    }

    dac_oneshot_config_t chan1_cfg = {
        .chan_id = DAC_CHAN_1,
    };
    err = dac_oneshot_new_channel(&chan1_cfg, &this->chan1_handle);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "dac chan1 create failed: %d", err);
      return;
    }

    minSampleInterval = DEFAULT_TICKS * 100 / vehicle->maxRpmPercentage;
    maxSampleInterval = DEFAULT_TICKS;
    variableTimerTicks = maxSampleInterval;
    appliedVariableTimerTicks = variableTimerTicks;

    // Interrupt timer for variable sample rate playback
    gptimer_config_t timer_config_variable = {};
    timer_config_variable.clk_src = GPTIMER_CLK_SRC_DEFAULT; // Select the default clock source
    timer_config_variable.direction = GPTIMER_COUNT_UP;      // Counting direction is up
    timer_config_variable.resolution_hz =
        TIMER_RESOLUTION_HZ; // Resolution is 4 MHz, 1 tick = 0.25 µs
    // Create a timer instance
    err = gptimer_new_timer(&timer_config_variable, &variableTimer);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "gptimer variable new failed: %d", err);
      return;
    }

    // Configure alarm (like timerAlarmWrite)
    gptimer_alarm_config_t alarm_config_variable = {};
    alarm_config_variable.alarm_count = DEFAULT_TICKS; // fixed rate, variable handled in software
    alarm_config_variable.reload_count = 0;            // start from 0
    alarm_config_variable.flags.auto_reload_on_alarm = true; // periodic mode
    err = gptimer_set_alarm_action(variableTimer, &alarm_config_variable);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "gptimer variable set_alarm failed: %d", err);
      return;
    }

    // Attach ISR (like timerAttachInterrupt)
    gptimer_event_callbacks_t cbs_variable = {
        .on_alarm = variable_timer_cb,
    };
    err = gptimer_register_event_callbacks(variableTimer, &cbs_variable, NULL);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "gptimer variable cb reg failed: %d", err);
      return;
    }

    // Enable + start timer (like timerAlarmEnable)
    err = gptimer_enable(variableTimer);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "gptimer variable enable failed: %d", err);
      return;
    }
    err = gptimer_start(variableTimer);
    if (err != ESP_OK) {
      ESP_LOGE("SND", "gptimer variable start failed: %d", err);
      return;
    }

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
    if (variableTimer) {
      // TODO delete timer
      variableTimer = nullptr;
    }
    if (fixedTimer) {
      // TODO delete timer
      fixedTimer = nullptr;
    }

    this->modelState = NULL;
    this->vehicle = NULL;
    this->config = NULL;
  }

  ModelState *modelState = NULL;
  Vehicle *vehicle = NULL;
  Config *config = NULL;

  // 128, but needs to be ramped up slowly to prevent popping noise, if switched on
  uint8_t dacOffset = 0;

  dac_oneshot_handle_t chan0_handle;
  dac_oneshot_handle_t chan1_handle;

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

// Handle deferred timer updates (safe to call from task context)
inline void updateTimerIfNeeded() {
  if (pendingTimerUpdate && variableTimer) {
    gptimer_alarm_config_t alarm_cfg = {};
    alarm_cfg.alarm_count = newTimerTicks;
    alarm_cfg.reload_count = 0;
    alarm_cfg.flags.auto_reload_on_alarm = true;

    esp_err_t err = gptimer_set_alarm_action(variableTimer, &alarm_cfg);
    if (err == ESP_OK) {
      appliedVariableTimerTicks = newTimerTicks;
      pendingTimerUpdate = false;
      ESP_LOGV("SND", "Timer updated to %d ticks", newTimerTicks);
    } else {
      ESP_LOGW("SND", "Failed to update timer: %d", err);
    }
  }
}

bool IRAM_ATTR fixed_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
                              void *user_ctx) {
  static uint32_t curHornSample = 0;         // Index of currently loaded horn sample
  static uint32_t curReversingSample = 0;    // Index of currently loaded reversing beep sample
  static uint32_t curIndicatorSample = 0;    // Index of currently loaded indicator tick sample
  static uint32_t curWastegateSample = 0;    // Index of currently loaded wastegate sample
  static uint32_t curBrakeSample = 0;        // Index of currently loaded brake sound sample
  static uint32_t curParkingBrakeSample = 0; // Index of currently loaded brake sound sample
  static uint32_t curShiftingSample = 0;     // Index of currently loaded shifting sample
  static uint32_t curDieselKnockSample = 0;  // Index of currently loaded Diesel knock sample
  static uint32_t curCouplingSample = 0;     // Index of currently loaded trailer coupling sample
  static uint32_t curUncouplingSample = 0;   // Index of currently loaded trailer uncoupling sample

  static int32_t a = 0, a1 = 0; // Input signals "a" for mixer
  static int32_t b = 0, b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0, b7 = 0, b8 = 0,
                 b9 = 0; // Input signals "b" for mixer

  static bool knockSilent = 0;         // This knock will be more silent
  static uint8_t curKnockCylinder = 0; // Index of currently ignited zylinder

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
    if (curIndicatorSample < sizeof(indicatorSamples) - 1) {
      b2 = (indicatorSamples[curIndicatorSample] * soundController.vehicle->indicatorVolume / 100);
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
  if (soundController.shifting &&
      soundController.modelState->isEngineRunning()) { // TODO -> vehicle parameters ?
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

  // Mixing sounds together
  a = a1;                                                      // Horn
  b = b0 * 5 + b1 + b2 / 2 + b3 + b4 + b5 + b6 + b7 + b8 + b9; // Other sounds

  // DAC output (groups a + b mixed together) - fixed rate sounds on DAC2
  uint8_t output = constrain(((a * 8 / 10) + (b * 2 / 10)) * soundController.config->volume / 100 +
                                 soundController.dacOffset,
                             0, 255);

  // Write to DAC2 (fixed timer handles horn/indicator/FX sounds on DAC2)
  if (soundController.chan1_handle) {
    dac_oneshot_output_voltage(soundController.chan1_handle, output);
  }

  return true;
}

bool IRAM_ATTR variable_timer_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata,
                                 void *user_ctx) {
  // We'll update the timer alarm after computing the new desired period
  static uint32_t attenuatorMillis = 0;
  static uint32_t curEngineSample = 0;       // Index of currently loaded engine sample
  static uint32_t curRevSample = 0;          // Index of currently loaded engine rev sample
  static uint32_t curTurboSample = 0;        // Index of currently loaded turbo sample
  static uint32_t curFanSample = 0;          // Index of currently loaded fan sample
  static uint32_t curChargerSample = 0;      // Index of currently loaded charger sample
  static uint32_t curStartSample = 0;        // Index of currently loaded start sample
  static uint32_t curJakeBrakeSample = 0;    // Index of currently loaded jake brake sample
  static uint32_t lastDieselKnockSample = 0; // Index of last Diesel knock sample
  static uint16_t attenuator = 0;            // Used for volume adjustment during engine switch off
  static uint16_t speedPercentage = 0;       // slows the engine down during shutdown

  // Input signals for mixer:
  // a = engine, b = additional, c = turbo, d = fan, e = supercharger
  static int32_t a = 0, a1 = 0, a2 = 0, a3 = 0, b = 0, c = 0, d = 0, e = 0;
  uint8_t a1Multi = 0; // Volume multipliers

  switch (soundController.modelState->getEngineState()) {
  case OFF:
    variableTimerTicks = DEFAULT_TICKS;

    a = 0; // volume = zero
    break;
  case STARTING:
    variableTimerTicks = DEFAULT_TICKS;

    if (curStartSample < sizeof(startSamples) - 1) {
      a = (startSamples[curStartSample] * soundController.throttleDependentVolume / 100 *
           soundController.vehicle->startVolume / 100);
      curStartSample++;
    } else {
      curStartSample = 0;
      soundController.modelState->setEngineState(RUNNING);
      soundController.modelState->setAirBrake(true);
    }
    break;
  case RUNNING:
    // Engine idle & revving sounds (mixed together according to engine rpm)
    // Match Arduino: always use variable rate derived from RPM
    variableTimerTicks = engineSampleRate; // our variable idle sampling rate!

    if (!soundController.modelState->isJakeBrake()) {
      if (curEngineSample < sizeof(samples) - 1) {
        // Idle sound
        a1 = samples[curEngineSample] * soundController.throttleDependentVolume / 100 *
             soundController.vehicle->idleVolume / 100;
        a3 = 0;
        curEngineSample++;

        if (soundController.vehicle->revSoundEnabled) {
          a2 = revSamples[curRevSample] * soundController.throttleDependentRevVolume / 100 *
               soundController.vehicle->revVolume / 100;
          if (curRevSample < sizeof(revSamples)) {
            curRevSample++;
          }
        }

        // Trigger throttle dependent Diesel ignition "knock" sound (played in the fixed sample rate
        // interrupt)
        if (curEngineSample - lastDieselKnockSample >
            (sizeof(samples) / soundController.vehicle->dieselKnockInterval)) {
          soundController.dieselKnockTrigger = true;
          soundController.dieselKnockTriggerFirst = false;
          lastDieselKnockSample = curEngineSample;
        }
      } else {
        curEngineSample = 0;
        if (soundController.modelState->isJakeBrake()) {
          soundController.engineJakeBraking = true;
        }
        curRevSample = 0;
        lastDieselKnockSample = 0;
        soundController.dieselKnockTrigger = true;
        soundController.dieselKnockTriggerFirst = true;
      }
      curJakeBrakeSample = 0;
    } else {
      if (soundController.vehicle->jakeBrakeEnabled) {
        a3 = jakeBrakeSamples[curJakeBrakeSample] * soundController.rpmDependentJakeBrakeVolume /
             100 * soundController.vehicle->jakeBrakeVolume / 100;
        a2 = 0;
        a1 = 0;
        if (curJakeBrakeSample < sizeof(jakeBrakeSamples) - 1) {
          curJakeBrakeSample++;
        } else {
          curJakeBrakeSample = 0;
          if (!soundController.modelState->isJakeBrake()) {
            soundController.engineJakeBraking = false;
          }
        }

        curEngineSample = 0;
        curRevSample = 0;
      }
    }

    // Engine sound mixer
    if (soundController.vehicle->revSoundEnabled) {
      // Mixing the idle and rev sounds together, according to engine rpm
      // Below the "revSwitchPoint" target, the idle volume precentage is 90%, then falling to 0% @
      // max. rpm. The total of idle and rev volume percentage is always 100%
      uint16_t rpm = soundController.modelState->getRPM();
      if (rpm > soundController.vehicle->revSwitchPoint) {
        a1Multi =
            map(rpm, soundController.vehicle->idleEndPoint, soundController.vehicle->revSwitchPoint,
                0, soundController.vehicle->idleVolumeProportion);
      } else {
        a1Multi = soundController.vehicle->idleVolumeProportion; // 90 - 100% proportion
      }
      if (rpm > soundController.vehicle->idleEndPoint) {
        a1Multi = 0;
      }

      a1 = a1 * a1Multi / 100;         // Idle volume
      a2 = a2 * (100 - a1Multi) / 100; // Rev volume

      a = a1 + a2 + a3; // Idle and rev sounds mixed together
    } else {
      a = a1 + a3; // Idle sound only
    }

    // Turbo sound
    if (soundController.vehicle->turboEnabled) {
      if (curTurboSample < sizeof(turboSamples) - 1) {
        c = (turboSamples[curTurboSample] * soundController.throttleDependentTurboVolume / 100 *
             soundController.vehicle->turboVolume / 100);
        curTurboSample++;
      } else {
        curTurboSample = 0;
      }
    }

    // Fan sound
    if (soundController.vehicle->fanEnabled) {
      if (curFanSample < sizeof(fanSamples) - 1) {
        d = (fanSamples[curFanSample] * soundController.throttleDependentFanVolume / 100 *
             soundController.vehicle->fanVolume / 100);
        curFanSample++;
      } else {
        curFanSample = 0;
      }
    }

    // Supercharger sound
    if (soundController.vehicle->chargerEnabled) {
      if (curChargerSample < sizeof(chargerSamples) - 1) {
        e = (chargerSamples[curChargerSample] * soundController.throttleDependentChargerVolume /
             100 * soundController.vehicle->chargerVolume / 100);
        curChargerSample++;
      } else {
        curChargerSample = 0;
      }
    }

    if (!soundController.ignition) {
      speedPercentage = 100;
      attenuator = 1;
      soundController.modelState->setEngineState(STOPPING);
    }

    break;
  case STOPPING:
    variableTimerTicks = DEFAULT_TICKS * speedPercentage / 100;

    if (curEngineSample < sizeof(samples) - 1) {
      a = (samples[curEngineSample] * soundController.throttleDependentVolume / 100 *
           soundController.vehicle->idleVolume / 100 / attenuator);
      curEngineSample++;
    } else {
      curEngineSample = 0;
    }

    // fade engine sound out
    if (millis() - attenuatorMillis > 100) { // Every 50ms
      attenuatorMillis = millis();
      attenuator++;          // attenuate volume
      speedPercentage += 20; // make it slower (10)
    }

    if (attenuator >= 50 || speedPercentage >= 500) { // 50 & 500
      a = 0;
      speedPercentage = 100;
      soundController.modelState->setEngineState(PARKING_BRAKE);
      soundController.modelState->setParkingBrake(true);
    }
    break;
  case PARKING_BRAKE:
    if (!soundController.modelState->isParkingBrake()) {
      soundController.modelState->setEngineState(OFF);
    }
    break;
  }

  // Apply updated variable timer period (Arduino's timerAlarmWrite equivalent)
  // Note: This is deferred to task context to avoid ISR issues
  if (variableTimerTicks != appliedVariableTimerTicks) {
    // Signal main task to update timer (safer than doing it in ISR)
    pendingTimerUpdate = true;
    newTimerTicks = variableTimerTicks;
  }

  // Mix signals, write directly to DAC (variable rate engine sounds)
  uint8_t output = constrain(((a * 8 / 10) + (b / 2) + (c / 5) + (d / 5) + (e / 5)) *
                                     soundController.config->volume / 100 +
                                 soundController.dacOffset,
                             0, 255);
  // Write to DAC1 (variable timer handles engine sounds on DAC1)
  if (soundController.chan0_handle) {
    dac_oneshot_output_voltage(soundController.chan0_handle, output);
  }

  return true;
}