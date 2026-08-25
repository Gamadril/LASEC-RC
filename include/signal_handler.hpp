#pragma once

#include "common.hpp"
#include "curves.hpp" // Nonlinear throttle curve arrays
#include "esc_controller.hpp"
#include "lights_controller.hpp"
#include "model_state.hpp"
#include "receiver_controller.hpp"
#include "servo_controller.hpp"
#include "sound_controller.hpp"
#include <cstring>

uint16_t mapThrottle(uint16_t ch_value, Config *config, ModelState *state) {
  // Input is around 1000 - 2000us, output 0-500 for forward and backwards
  uint16_t th_value = 0;

  // Auto throttle while gear shifting (synchronizing the Tamiya 3 speed gearbox)
  if (config->vehicle.shiftingAutoThrottle &&
      (state->getDriveState() == DRIVING_FORWARD || state->getDriveState() == BRAKING_FORWARD)) {
    if (state->getGearShift() == UP) {
      th_value = MIN_THROTTLE;
    } else if (state->getGearShift() == DOWN) {
      th_value = MAX_THROTTLE;
    }
  } else if (ch_value > config->escConfig.neutral) {
    th_value = map(ch_value, config->escConfig.neutral, config->escConfig.max, MIN_THROTTLE, MAX_THROTTLE);
  } else if (ch_value < config->escConfig.neutral) {
    th_value = map(ch_value, config->escConfig.min, config->escConfig.neutral, MIN_THROTTLE, MAX_THROTTLE);
  }

  return th_value;
}

void setupSignalHandlers(ModelState *state, LightsController *lights, SoundController *sound, EscController *esc,
                         ReceiverController *rc, ServoController *servo, Config *config) {
  static const char *TAG = "SIGH";

  state->onTurnIndicator.connect([lightsPtr = lights, soundPtr = sound](TurnSignal value) {
    bool tl = value == T_LEFT;
    bool tr = value == T_RIGHT;
    ESP_LOGI(TAG, "TURN: %s", tl ? "LEFT" : tr ? "RIGHT" : "OFF");
    if (lightsPtr) {
      if (tl) {
        lightsPtr->setTurnLeft(true);
        lightsPtr->setTurnRight(false);
      } else if (tr) {
        lightsPtr->setTurnLeft(false);
        lightsPtr->setTurnRight(true);
      } else {
        lightsPtr->setTurnLeft(false);
        lightsPtr->setTurnRight(false);
      }
    }
    soundPtr->onTurnSignal(tl || tr);
  });

  state->onHazard.connect([lightsPtr = lights, soundPtr = sound](bool value) {
    ESP_LOGI(TAG, "HAZARD: %s", value ? "ON" : "OFF");
    if (lightsPtr) {
      lightsPtr->setHazard(value);
    }
    soundPtr->onTurnSignal(value);
  });

  state->onFailSafe.connect([statePtr = state](bool value) {
    ESP_LOGI(TAG, "FAILSAFE: %s", value ? "ON" : "OFF");
    if (value) {
      statePtr->setHazard(value);
    }
  });

  state->onEngineState.connect([](EngineState value) {
    std::string state_str = es2str(value);
    ESP_LOGI(TAG, "ENGINE: %s", state_str.c_str());
  });

  state->onThrottle.connect([soundPtr = sound](uint16_t value) {
    ESP_LOGI(TAG, "THROTTLE: %d", value);
    soundPtr->onThrottleChange(value);
  });

  state->onHorn.connect([soundPtr = sound](bool value) {
    ESP_LOGI(TAG, "HORN: %s", value ? "ON" : "OFF");
    soundPtr->onHorn(value);
  });

  state->onGearShift.connect([soundPtr = sound](GearShift value) {
    ESP_LOGI(TAG, "GEAR_SHIFT: %s", value == UP ? "UP" : value == DOWN ? "DOWN" : "NOT");
    if (value != NOT) {
      soundPtr->onShifting(true);
    }
  });

  state->onGear.connect([statePtr = state](uint8_t previous, uint8_t current, bool clutch) {
    ESP_LOGI(TAG, "GEAR: %d -> %d, CLUTCH: %s", previous, current, clutch ? "ON" : "OFF");
    if (previous < current) {
      statePtr->setGearShift(UP);
    } else if (previous > current) {
      statePtr->setGearShift(DOWN);
    }
  });

  rc->onFrame.connect([statePtr = state, configPtr = config, servoPtr = servo](uint16_t *channels,
                                                                               uint8_t channels_count, bool failsafe) {
    if (!statePtr)
      return; // safety guard
    uint16_t value;

    // Copy channel values to state
    statePtr->setChannelsCount(channels_count);
    statePtr->setChannelValues(channels, channels_count);
    
    statePtr->setFailsafe(failsafe);
    if (failsafe) {
      return;
    }

    // check throttle
    value = channels[CH_THROTTLE - 1];
    statePtr->setThrottleRaw(value);
    statePtr->setThrottle(mapThrottle(value, configPtr, statePtr));
    if (statePtr->getEngineState() == OFF && value > 1700) {
      statePtr->setEngineState(STARTING);
    } else if (statePtr->getEngineState() == RUNNING && statePtr->getDriveState() == STANDING && value < 1300) {
      statePtr->setEngineState(STOPPING);
    }

    // check steering
    value = channels[CH_STEERING - 1];
    if (value == 0) {
      value = configPtr->steeringServo.neutral;
    }
    if (servoPtr) {
      servoPtr->set_steering_us(value);
    }
    TurnSignal ts = T_OFF;
    if (value > configPtr->steeringServo.neutral + configPtr->vehicle.indicatorOnThreshold) {
      ts = T_LEFT;
    } else if (value < configPtr->steeringServo.neutral - configPtr->vehicle.indicatorOnThreshold) {
      ts = T_RIGHT;
    }
    statePtr->setTurnSignal(ts);

    // check shifting
    value = channels[CH_SHIFTING - 1];
    if (value == 0) {
      value = configPtr->shiftingServo.neutral;
    }
    if (servoPtr) {
      servoPtr->set_shifting_us(value);
    }
    if (value > 1700) {
      statePtr->setGear(3);
    } else if (value < 1300) {
      statePtr->setGear(1);
    } else {
      statePtr->setGear(2);
    }

    // check clutch
    value = channels[CH_GEAR_CLUTCH - 1];
    if (value == 0) {
      value = 1500;
    }
    statePtr->setClutch(value > 1600);

    // check hazard
    value = channels[CH_HAZARD - 1];
    if (value == 0 || value > 1500) {
      statePtr->setHazard(true);
    } else {
      statePtr->setHazard(false);
    }

    // check horn
    value = channels[CH_HORN - 1];
    statePtr->setHorn(value != 0 && value > 1600);
  });
}

void engineMassSimulation(Config *config, ModelState *state, SoundController *soundController,
                          EscController *escController) {
  static int32_t targetRpm = 0; // The engine RPM target
  static ulong throtMillis = 0;
  uint8_t timeBase = 2;
  static bool clutchDisengaged = false;

  if (millis() - throtMillis > timeBase) { // Every 2ms
    throtMillis = millis();

    // Virtual clutch
    if ((state->getSpeed() < config->vehicle.clutchEngagingPoint && state->getRPM() < 250) ||
        state->getGearShift() != NOT || !state->isClutch()) {
      clutchDisengaged = true;
    } else {
      clutchDisengaged = false;
    }

    // Transmissions
    // Manual transmission ----
    if (clutchDisengaged) { // Clutch disengaged: Engine revving allowed
      targetRpm = reMap(curveLinear, state->getThrottle());
    } else { // Clutch engaged: Engine rpm synchronized with ESC power (speed)
      targetRpm = reMap(curveLinear, state->getSpeed());
    }

    // Engine RPM
    if (escController->isBraking() && state->getSpeed() < config->vehicle.clutchEngagingPoint) {
      targetRpm = 0; // keep engine @idle rpm, if braking at very low speed
    }

    // Accelerate engine
    if (targetRpm > (state->getRPM() + config->vehicle.acc) && (state->getRPM() + config->vehicle.acc) < MAX_RPM &&
        state->getEngineState() == RUNNING) {
      if (!state->isAirBrake()) { // No acceleration, if brake release noise still playing
        if (state->getGearShift() == DOWN) {
          state->setRPM(state->getRPM() + config->vehicle.acc / 2); // less aggressive rpm rise while downshifting
        } else {
          state->setRPM(state->getRPM() + config->vehicle.acc);
        }
      }
    }

    // Decelerate engine
    if (targetRpm < state->getRPM()) {
      auto newRpm = state->getRPM() - config->vehicle.dec;
      if (newRpm < MIN_RPM) {
        state->setRPM(MIN_RPM);
      } else {
        state->setRPM(newRpm);
      }
    }

    // Speed (sample rate) output
    soundController->onRpmChange();
  }
}
