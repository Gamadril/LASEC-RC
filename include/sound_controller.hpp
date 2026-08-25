#pragma once

#include <functional>

#include "config_handler.hpp"
#include "hal/sound_output.hpp"
#include "hal/wav_reader.hpp"
#include "model_state.hpp"
#include "sound_types.hpp"
#include "utils.hpp"
#include "vehicle.hpp"

#define AUDIO_RATE 44100
#define DEFAULT_TICKS (4000000 / AUDIO_RATE) // timer ticks per sample at 4MHz
#define TIMER_RESOLUTION_HZ 4000000          // 4 MHz → 0.25 µs per tick

class SoundController {
private:
  ModelState *_modelState = nullptr;
  Config *_config = nullptr;

  volatile uint16_t _engineSampleRate = 0;
  uint32_t _maxSampleInterval;
  uint32_t _minSampleInterval;

  uint16_t _currentThrottleFaded = 0;
  uint16_t _throttleDependentVolume = 0;
  uint16_t _throttleDependentTurboVolume = 0;
  uint16_t _tdFanVolume = 0;
  uint16_t _throttleDependentChargerVolume = 0;
  uint16_t _throttleDependentRevVolume = 0;
  uint16_t _throttleDependentKnockVolume = 0;
  uint16_t _rpmDependentKnockVolume = 0;
  uint16_t _rpmDependentJakeBrakeVolume = 0;
  uint16_t _rpmDependentWastegateVolume = 0;

  uint32_t _wastegateMillis = 0;

  bool _engineJakeBraking = false;
  bool _dieselKnockTrigger = false;
  bool _dieselKnockTriggerFirst = false;

  bool _turnSignal = false;
  bool _horn = false;
  bool _wastegate = false;
  bool _coupling = false;
  bool _uncoupling = false;
  bool _shifting = false;

  SoundOutput &_output;
  std::unordered_map<SoundType, WavReader *> &_sounds;

  void _getSample(AudioSample *sample) {
    int32_t a1 = 0; // Group "a" mixer signal (horn)
    int32_t b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0, b7 = 0, b8 = 0,
            b10 = 0; // Group "b" mixer signals

    // Diesel knock state
    static uint32_t lastKnockPhase = 0;
    static bool knockSilent = false;
    static uint8_t curKnockCylinder = 0;

    int32_t leftChannelValue = 0;
    int32_t rightChannelValue = 0;

    int16_t next_sample = 0;
    WavReader *effect_sound;

    // Group "a" (horn)
    effect_sound = _sounds[SoundType::HORN];
    if (_horn || (!effect_sound->is_start() && !effect_sound->is_end())) {
      next_sample = effect_sound->read_next_sample(_horn);
      a1 = next_sample * _config->vehicle.hornVolume / 100;
    }
    // ESP_LOGI("SC", "a1: %d", a1);

    // Group "b" (other sounds)

    // Reversing beep sound "b1"
    effect_sound = _sounds[SoundType::REVERSE];
    if (_modelState->isMovingBackward()) {
      next_sample = effect_sound->read_next_sample();
      b1 = next_sample * _config->vehicle.reversingVolume / 100;
    } else {
      effect_sound->reset();
      b1 = 0;
    }

    // turn signal tick sound "b2"
    effect_sound = _sounds[SoundType::TURN_SIGNAL];
    if (_turnSignal) {
      next_sample = effect_sound->read_next_sample();
      b2 = next_sample * _config->vehicle.indicatorVolume / 100;
    }

    // Wastegate (blowoff) sound, triggered after rapid throttle drop "b3"
    /*
    if (_config->vehicle.wastegateEnabled) {
      if (_wastegate) {
        if (curWastegateSample < wastegate_samples_length - 1) {
          b3 = (wastegate_samples[curWastegateSample] * _rpmDependentWastegateVolume
    / 100 * _vehicle->wastegateVolume / 100); curWastegateSample++; } else {
          _wastegate = false;
        }
      } else {
        b3 = 0;
        curWastegateSample = 0; // ensure, next sound will start @ first sample
      }
    }
    */

    // Air brake release sound "b4", triggered after stop
    effect_sound = _sounds[SoundType::AIR_BRAKE];
    if (_modelState->isAirBrake()) {
      if (!effect_sound->is_end()) {
        next_sample = effect_sound->read_next_sample();
        b4 = next_sample * _config->vehicle.brakeVolume / 100;
      } else {
        _modelState->setAirBrake(false);
        effect_sound->reset();
      }
    }

    // Pneumatic gear shifting sound "b6", triggered while shifting the TAMIYA 3 speed transmission
    effect_sound = _sounds[SoundType::GEAR_SHIFT];
    if (_shifting) {
      if (!effect_sound->is_end()) {
        next_sample = effect_sound->read_next_sample();
        b6 = next_sample * _config->vehicle.shiftingVolume / 100;
      } else {
        _shifting = false;
        effect_sound->reset();
      }
    }

    /*
    if (_dieselKnockTriggerFirst) {
      _dieselKnockTriggerFirst = false;
      curKnockCylinder = 0;
    }

    if (_dieselKnockTrigger) {
      _dieselKnockTrigger = false;
      curKnockCylinder++; // Count ignition sequence
      curDieselKnockSample = 0;
    }

    knockSilent = true;
    for (uint8_t i = 0; i < sizeof(_config->vehicle.dieselKnockCylinders); i++) {
      if (_config->vehicle.dieselKnockCylinders[i] == curKnockCylinder) {
        knockSilent = false;
        break;
      }
    }

    if (curDieselKnockSample < knock_samples_length) {
      b7 = knock_samples[curDieselKnockSample] * _config->vehicle.dieselKnockVolume / 100 *
           _throttleDependentKnockVolume / 100;

      if (_config->vehicle.dieselKnockDependsOnRPM) {
        b7 = b7 * _rpmDependentKnockVolume / 100;
      }

      curDieselKnockSample++;
      if (knockSilent) {
        // changing knock volume according to engine type and cylinder!
        b7 = b7 * _config->vehicle.dieselKnockAdaptiveVolume / 100;
      }
    }
    */

    if (_config->vehicle.couplingSoundEnabled) {
      // Trailer coupling sound "b8"
      if (_coupling) {
        effect_sound = _sounds[SoundType::COUPLING];
        if (!effect_sound->is_end()) {
          next_sample = effect_sound->read_next_sample();
          b8 = next_sample * _config->vehicle.couplingVolume / 100;
        } else {
          _coupling = false;
          effect_sound->reset();
        }
      }

      if (_uncoupling) {
        effect_sound = _sounds[SoundType::DECOUPLING];
        if (!effect_sound->is_end()) {
          next_sample = effect_sound->read_next_sample();
          b8 = next_sample * _config->vehicle.couplingVolume / 100;
        } else {
          _uncoupling = false;
          effect_sound->reset();
        }
      }
    }

    // Diesel knock sound - triggered by phase accumulator in engine resampler
    /*
    if (_dieselKnockTrigger) {
      if (curDieselKnockSample < knock_samples_length - 1) {
        if (_dieselKnockTriggerFirst) {
          b10 = (knock_samples[curDieselKnockSample] * _throttleDependentKnockVolume
    / 100 * _config->vehicle.dieselKnockVolume / 100); } else { if (knockSilent) { b10 =
                (knock_samples[curDieselKnockSample] * _throttleDependentKnockVolume
    / 100 * _config->vehicle.dieselKnockVolume / 100 / 3); } else { b10 =
                (knock_samples[curDieselKnockSample] * _throttleDependentKnockVolume
    / 100 * _config->vehicle.dieselKnockVolume / 100);
          }
        }
        curDieselKnockSample++;
      } else {
        _dieselKnockTrigger = false;
        curDieselKnockSample = 0;
        curKnockCylinder++;
        // dieselKnockCylinders is an array, check if current cylinder value is non-zero
        if (curKnockCylinder >= 4 ||
            _config->vehicle.dieselKnockCylinders[curKnockCylinder] == 0) {
          curKnockCylinder = 0;
        }
        knockSilent = !knockSilent;
      }
    } else {
      b10 = 0;
    }
    */

    // Resample engine at fixed I2S rate using 16.16 phase accumulators
    static uint32_t phaseIdle = 0;
    static uint32_t phaseRev = 0;
    static uint32_t phaseJake = 0;
    static uint32_t phaseTurbo = 0;
    static uint32_t phaseFan = 0;
    static uint32_t phaseCharger = 0;
    static EngineState prevEs = OFF;
    EngineState es = _modelState->getEngineState();
    if (es != prevEs) {
      // Reset phases on state transition for clean starts
      if (es == RUNNING) {
        phaseIdle = phaseRev = phaseJake = 0;
        lastKnockPhase = 0;
        _dieselKnockTrigger = true;
        _dieselKnockTriggerFirst = true;
      }
      prevEs = es;
    }

    if (es == STARTING) {
      effect_sound = _sounds[SoundType::ENGINE_START];

      if (!effect_sound->is_end()) {
        next_sample = effect_sound->read_next_sample();
      } else {
        _modelState->setEngineState(RUNNING);
        _modelState->setAirBrake(true);
        effect_sound->reset();
      }

      int32_t startScaled = next_sample * _config->vehicle.startVolume / 100;
      leftChannelValue = (int16_t)constrain(startScaled, -32768, 32767);
    } else if (es == RUNNING) {
      // Use freshest RPM-derived ticks to minimize latency
      uint32_t ticks = _engineSampleRate ? _engineSampleRate : DEFAULT_TICKS;
      uint32_t phaseInc = (uint32_t)(((uint64_t)TIMER_RESOLUTION_HZ << 16) / ((uint64_t)ticks * (uint64_t)AUDIO_RATE));

      int32_t engineMixed = 0;

      if (!_modelState->isJakeBrake()) {
        // Idle
        effect_sound = _sounds[SoundType::ENGINE_IDLE];
        phaseIdle += phaseInc;
        // Wrap phase to prevent overflow
        uint32_t maxPhaseIdle = effect_sound->get_samples_count() << 16;
        if (phaseIdle >= maxPhaseIdle)
          phaseIdle -= maxPhaseIdle;

        int32_t idleInterp = 0;
        uint32_t i = (phaseIdle >> 16) % effect_sound->get_samples_count();

        // Check if we wrapped around - trigger jake brake if needed
        if (i == 0 && _modelState->isJakeBrake()) {
          _engineJakeBraking = true;
        }

        uint32_t frac = phaseIdle & 0xFFFF;
        int16_t s0 = effect_sound->read_sample(i);
        int16_t s1 = effect_sound->read_next_sample();
        idleInterp = s0 + ((int32_t)(s1 - s0) * (int32_t)frac >> 16);

        // Diesel knock triggering based on phase position
        uint32_t knockInterval = (effect_sound->get_samples_count() << 16) / _config->vehicle.dieselKnockInterval;
        uint32_t phaseDiff =
            (phaseIdle >= lastKnockPhase) ? (phaseIdle - lastKnockPhase) : (maxPhaseIdle - lastKnockPhase + phaseIdle);
        if (phaseDiff >= knockInterval) {
          _dieselKnockTrigger = true;
          _dieselKnockTriggerFirst = (lastKnockPhase == 0);
          lastKnockPhase = phaseIdle;
        }

        // Rev (optional)
        int32_t revInterp = 0;
        if (_config->vehicle.revSoundEnabled) {
          effect_sound = _sounds[SoundType::ENGINE_REV];
          phaseRev += phaseInc;
          // Wrap phase to prevent overflow
          uint32_t maxPhaseRev = effect_sound->get_samples_count() << 16;
          if (phaseRev >= maxPhaseRev)
            phaseRev -= maxPhaseRev;

          uint32_t i = (phaseRev >> 16) % effect_sound->get_samples_count();
          uint32_t frac = phaseRev & 0xFFFF;
          int16_t r0 = effect_sound->read_sample(i);
          int16_t r1 = effect_sound->read_next_sample();
          revInterp = r0 + ((int32_t)(r1 - r0) * (int32_t)frac >> 16);
        }

        uint16_t rpm = _modelState->getRPM();
        uint8_t a1Multi;
        if (rpm > _config->vehicle.revSwitchPoint) {
          a1Multi = map(rpm, _config->vehicle.idleEndPoint, _config->vehicle.revSwitchPoint, 0,
                        _config->vehicle.idleVolumeProportion);
        } else {
          a1Multi = _config->vehicle.idleVolumeProportion;
        }
        if (rpm > _config->vehicle.idleEndPoint) {
          a1Multi = 0;
        }

        int32_t idleScaled = idleInterp;
        idleScaled = (idleScaled * _throttleDependentVolume) / 100;
        idleScaled = (idleScaled * _config->vehicle.idleVolume) / 100;
        idleScaled = (idleScaled * a1Multi) / 100;

        int32_t revScaled = revInterp;
        revScaled = (revScaled * _throttleDependentRevVolume) / 100;
        revScaled = (revScaled * _config->vehicle.revVolume) / 100;
        revScaled = (revScaled * (100 - a1Multi)) / 100;

        engineMixed = idleScaled + revScaled;
      } else if (_config->vehicle.jakeBrakeEnabled) {
        effect_sound = _sounds[SoundType::ENGINE_REV];
        phaseJake += phaseInc;

        uint32_t maxPhase = effect_sound->get_samples_count() << 16;
        if (phaseJake >= maxPhase)
          phaseJake -= maxPhase;

        int32_t jakeInterp = 0;
        uint32_t i = (phaseJake >> 16) % effect_sound->get_samples_count();

        // Check if jake brake sample finished
        if (effect_sound->is_end()) {
          phaseJake = 0;
          if (!_modelState->isJakeBrake()) {
            _engineJakeBraking = false;
          }
        }

        uint32_t frac = phaseJake & 0xFFFF;
        int16_t j0 = effect_sound->read_sample(i);
        int16_t j1 = effect_sound->read_next_sample();
        jakeInterp = j0 + ((int32_t)(j1 - j0) * (int32_t)frac >> 16);

        // Jake volume
        engineMixed = jakeInterp;
        engineMixed = (engineMixed * _rpmDependentJakeBrakeVolume) / 100;
        engineMixed = (engineMixed * _config->vehicle.jakeBrakeVolume) / 100;

        // Reset idle/rev phases while jake braking
        phaseIdle = 0;
        phaseRev = 0;
      }

      // Turbo sound (resampled at engine rate)
      int32_t turboRaw = 0;
      if (_config->vehicle.turboEnabled) {
        effect_sound = _sounds[SoundType::ENGINE_TURBO];
        phaseTurbo += phaseInc;

        uint32_t maxPhase = effect_sound->get_samples_count() << 16;
        if (phaseTurbo >= maxPhase)
          phaseTurbo -= maxPhase;

        uint32_t i = (phaseTurbo >> 16) % effect_sound->get_samples_count();
        uint32_t frac = phaseTurbo & 0xFFFF;
        int16_t t0 = effect_sound->read_sample(i);
        int16_t t1 = effect_sound->read_next_sample();
        int32_t turboInterp = t0 + ((int32_t)(t1 - t0) * (int32_t)frac >> 16);

        turboRaw = turboInterp * _throttleDependentTurboVolume / 100;
        turboRaw = turboRaw * _config->vehicle.turboVolume / 100;
      }

      int32_t fan = 0;
      if (_config->vehicle.fanEnabled && es == RUNNING) {
        effect_sound = _sounds[SoundType::ENGINE_FAN];
        next_sample = effect_sound->read_next_sample();
        fan = next_sample * _tdFanVolume / 100 * _config->vehicle.fanVolume / 100;
      } else {
        effect_sound->reset();
      }

      // Supercharger sound (resampled at engine rate)
      int32_t chargerRaw = 0;
      /*
      if (_config->vehicle.chargerEnabled) {
        phaseCharger += phaseInc;
        if (lenCharger > 1) {
          // Wrap phase to prevent overflow
          uint32_t maxPhase = lenCharger << 16;
          if (phaseCharger >= maxPhase)
            phaseCharger -= maxPhase;

          uint32_t i6 = (phaseCharger >> 16) % lenCharger;
          uint32_t j6 = (i6 + 1) % lenCharger;
          uint32_t frac6 = (phaseCharger & 0xFFFF);
          int16_t c0 = chargerSamples[i6];
          int16_t c1 = chargerSamples[j6];
          int32_t chargerInterp = c0 + ((int32_t)(c1 - c0) * (int32_t)frac6 >> 16);
          chargerRaw = (chargerInterp >> 8) * _throttleDependentChargerVolume / 100 *
                       _config->vehicle.chargerVolume / 100;
        }
      }
      */

      int32_t mixed = ((engineMixed * 8 / 10) + (turboRaw / 5) + (fan / 5) + (chargerRaw / 5));
      int32_t totalMixed = (mixed * _config->volume) / 100;
      leftChannelValue = constrain(totalMixed, -32768, 32767);
    } else if (es == STOPPING) {
      effect_sound = _sounds[SoundType::ENGINE_STOP];

      if (!effect_sound->is_end()) {
        next_sample = effect_sound->read_next_sample();
      } else {
        _modelState->setEngineState(PARKING_BRAKE);
        effect_sound->reset();
      }

      int32_t stoppingScaled = next_sample;
      stoppingScaled = (stoppingScaled * _config->vehicle.idleVolume) / 100;
      stoppingScaled = (stoppingScaled * _config->volume) / 100;
      leftChannelValue = constrain(stoppingScaled, -32768, 32767);
    } else if (es == PARKING_BRAKE) {
      effect_sound = _sounds[SoundType::PARKING_BRAKE];

      if (!effect_sound->is_end()) {
        next_sample = effect_sound->read_next_sample();
      } else {
        _modelState->setEngineState(OFF);
        effect_sound->reset();
      }

      int32_t parkingBrakeScaled = next_sample;
      parkingBrakeScaled = (parkingBrakeScaled * _config->vehicle.parkingBrakeVolume) / 100;
      parkingBrakeScaled = (parkingBrakeScaled * _config->volume) / 100;
      leftChannelValue = constrain(parkingBrakeScaled, -32768, 32767);
    } else {
      leftChannelValue = 0;
    }

    // Mix group "a" and "b" sounds (horn, reversing, indicators, etc.) for RIGHT channel
    // These sounds play regardless of engine state
    int32_t groupA = (a1 * 8 / 10);
    int32_t groupB = (b1 + b2 + b3 + b4 + b5 + b6 + b7 + b8 + b10) / 2;
    int32_t rightMixed = (groupA + groupB) * _config->volume / 100;
    rightChannelValue = constrain(rightMixed, -32768, 32767);

    sample->left = leftChannelValue;
    sample->right = rightChannelValue;
  }

  // Static wrapper function for IRAM callback - calls member function
  static void _get_sample_static(AudioSample *sample, void *user_data) {
    SoundController *instance = static_cast<SoundController *>(user_data);
    if (instance) {
      instance->_getSample(sample);
    }
  }

public:
  SoundController(SoundOutput &sound_output, std::unordered_map<SoundType, WavReader *> &sounds)
      : _output(sound_output), _sounds(sounds) {
  }

  void reinit() {
  }

  void onShifting(bool value) {
    if (!_shifting && value) {
      _sounds[SoundType::GEAR_SHIFT]->reset();
    }
    _shifting = value;
  }

  void onTurnSignal(bool value) {
    if (!_turnSignal && value) {
      _sounds[SoundType::TURN_SIGNAL]->reset();
    }
    _turnSignal = value;
  }

  void onHorn(bool value) {
    if (!_horn && value) {
      _sounds[SoundType::HORN]->reset();
    }
    _horn = value;
  }

  void onCoupling() {
    _coupling = true;
  }

  void onUncoupling() {
    _uncoupling = true;
  }

  void onRpmChange() {
    uint16_t value = _modelState->getRPM();
    _engineSampleRate = map(value, MIN_RPM, MAX_RPM, _maxSampleInterval, _minSampleInterval); // Idle

    // Calculate RPM dependent Diesel knock volume
    if (_config->vehicle.dieselKnockDependsOnRPM) {
      if (value > 400) {
        _rpmDependentKnockVolume = map(value, 400, MAX_RPM, 5, 100);
      } else {
        _rpmDependentKnockVolume = 5;
      }
    }

    // Calculate engine rpm dependent jake brake volume
    if (_config->vehicle.jakeBrakeEnabled) {
      if (_modelState->isEngineRunning()) {
        _rpmDependentJakeBrakeVolume = map(value, MIN_RPM, MAX_RPM, _config->vehicle.jakeBrakeIdleVolume, 100);
      } else {
        _rpmDependentJakeBrakeVolume = _config->vehicle.jakeBrakeIdleVolume;
      }
    }

    // Calculate engine rpm dependent turbo volume
    if (_config->vehicle.turboEnabled) {
      if (_modelState->isEngineRunning()) {
        _throttleDependentTurboVolume = map(value, MIN_RPM, MAX_RPM, _config->vehicle.turboIdleVolume, 100);
      } else {
        _throttleDependentTurboVolume = _config->vehicle.turboIdleVolume;
      }
    }

    // Calculate engine rpm dependent cooling fan volume
    if (_config->vehicle.fanEnabled) {
      if (_modelState->isEngineRunning() && (value > _config->vehicle.fanStartPoint)) {
        _tdFanVolume = map(value, _config->vehicle.fanStartPoint, MAX_RPM, _config->vehicle.fanIdleVolume, 100);
      } else {
        _tdFanVolume = _config->vehicle.fanIdleVolume;
      }
    }

    // Calculate throttle dependent supercharger volume
    if (_config->vehicle.chargerEnabled) {
      if (_modelState->isEngineRunningNotBreaking() && (value > _config->vehicle.chargerStartPoint))
        _throttleDependentChargerVolume = map(_currentThrottleFaded, _config->vehicle.chargerStartPoint, MAX_RPM,
                                              _config->vehicle.chargerIdleVolume, 100);
      else
        _throttleDependentChargerVolume = _config->vehicle.chargerIdleVolume;
    }

    // Calculate engine rpm dependent wastegate volume
    if (_config->vehicle.wastegateEnabled) {
      if (_modelState->isEngineRunning()) {
        _rpmDependentWastegateVolume = map(value, MIN_RPM, MAX_RPM, _config->vehicle.wastegateIdleVolume, 100);
      } else {
        _rpmDependentWastegateVolume = _config->vehicle.wastegateIdleVolume;
      }
    }
  }

  void onThrottleChange(uint16_t value) {
    static unsigned long throttleFaderMicros = 0;
    static uint16_t lastThrottle = 0;

    if (_config->vehicle.wastegateEnabled) {
      // Prevent Wastegate from being triggered while downshifting
      if (_modelState->getGearShift() == DOWN) {
        _wastegateMillis = millis();
      }

      // Trigger Wastegate, if throttle rapidly dropped
      if (lastThrottle - value > 70 && !_modelState->isBraking() && millis() - _wastegateMillis > 1000) {
        _wastegateMillis = millis();
        _wastegate = true;
      }
    }

    if (micros() - throttleFaderMicros > 500) { // Every 0.5ms
      throttleFaderMicros = micros();

      if (_currentThrottleFaded < value && _currentThrottleFaded < 499) {
        _currentThrottleFaded += 32; // Very fast fade-in
        if (_currentThrottleFaded > value)
          _currentThrottleFaded = value;
      } else if (_currentThrottleFaded > value && _currentThrottleFaded > 32) {
        _currentThrottleFaded -= 32; // Very fast fade-out
        if (_currentThrottleFaded < value)
          _currentThrottleFaded = value;
      }

      // Calculate throttle dependent engine idle volume
      if (_modelState->isEngineRunningNotBreaking()) {
        _throttleDependentVolume =
            map(_currentThrottleFaded, 0, 500, _config->vehicle.engineIdleVolume, _config->vehicle.fullThrottleVolume);
      } else {
        _throttleDependentVolume = _config->vehicle.engineIdleVolume;
      }

      // Calculate throttle dependent engine rev volume
      if (_config->vehicle.revSoundEnabled) {
        if (_modelState->isEngineRunningNotBreaking()) {
          _throttleDependentRevVolume =
              map(_currentThrottleFaded, 0, 500, _config->vehicle.engineRevVolume, _config->vehicle.fullThrottleVolume);
        } else {
          _throttleDependentRevVolume = _config->vehicle.engineRevVolume;
        }
      }

      // Calculate throttle dependent Diesel knock volume
      if (_modelState->isEngineRunningNotBreaking() &&
          (_currentThrottleFaded > _config->vehicle.dieselKnockStartPoint)) {
        _throttleDependentKnockVolume = map(_currentThrottleFaded, _config->vehicle.dieselKnockStartPoint, 500,
                                            _config->vehicle.dieselKnockIdleVolume, 100);
      } else {
        _throttleDependentKnockVolume = _config->vehicle.dieselKnockIdleVolume;
      }
    }
    lastThrottle = value;
  }

  void init(ModelState *modelState, Config *config) {
    this->_modelState = modelState;
    this->_config = config;

    _minSampleInterval = DEFAULT_TICKS * 100 / _config->vehicle.maxRpmPercentage;
    _maxSampleInterval = DEFAULT_TICKS;

    // Initialize engine sample rate based on current RPM (typically 0 at startup = idle)
    uint16_t initialRpm = _modelState ? _modelState->getRPM() : 0;
    _engineSampleRate = map(initialRpm, MIN_RPM, MAX_RPM, _maxSampleInterval, _minSampleInterval);

    _output.setSampleCallback(_get_sample_static, this);
    _output.init(AUDIO_RATE);
  }

  void deinit() {
    _output.deinit();
    this->_modelState = nullptr;
    this->_config = nullptr;
  }
};
