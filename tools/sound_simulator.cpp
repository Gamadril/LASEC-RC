#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <vector>

// PortAudio for cross-platform audio
#include <portaudio.h>

// ncurses for terminal UI
#include <ncurses.h>

#define AUDIO_RATE 44100
#define DEFAULT_TICKS (4000000 / AUDIO_RATE)
#define TIMER_RESOLUTION_HZ 4000000
#define MIN_RPM 0
#define MAX_RPM 500

// Constrain function (if not defined)
#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif

// Use actual headers
#include "model_state.h"
#include "vehicles/actros_1851.h"

// Use actual Vehicle and Config from headers
// Config is in config_handler.h, but we'll create a simple one for simulator
struct SimConfig {
  uint8_t volume = 100;
};

// Map function
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Time functions
unsigned long millis() {
  static auto start = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

unsigned long micros() {
  static auto start = std::chrono::steady_clock::now();
  auto now = std::chrono::steady_clock::now();
  return std::chrono::duration_cast<std::chrono::microseconds>(now - start).count();
}

// Global state
ModelState modelState;
Vehicle vehicle = ACTROS_1851; // Use default vehicle
SimConfig config;

// Sound controller state (simplified version matching the ISR logic)
struct SoundControllerState {
  uint32_t minSampleInterval;
  uint32_t maxSampleInterval;
  volatile uint32_t engineSampleRate;

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

  bool horn = false;
  bool hornLatch = false;
  bool turnSignal = false;
  bool wastegate = false;
  bool coupling = false;
  bool uncoupling = false;
  bool shifting = false;
  bool dieselKnockTrigger = false;
  bool dieselKnockTriggerFirst = false;
  bool ignition = false;

  // Phase accumulators (16.16 fixed point)
  uint32_t phaseIdle = 0;
  uint32_t phaseRev = 0;
  uint32_t phaseJake = 0;
  uint32_t phaseStart = 0;
  uint32_t phaseTurbo = 0;
  uint32_t phaseFan = 0;
  uint32_t phaseCharger = 0;
  uint32_t phaseStop = 0;

  // Sample indices for fixed-rate sounds
  uint32_t curHornSample = 0;
  uint32_t curReversingSample = 0;
  uint32_t curIndicatorSample = 0;
  uint32_t curWastegateSample = 0;
  uint32_t curBrakeSample = 0;
  uint32_t curParkingBrakeSample = 0;
  uint32_t curShiftingSample = 0;
  uint32_t curDieselKnockSample = 0;
  uint32_t curKnockCylinder = 0;
  uint32_t lastKnockPhase = 0;

  // Stopping state
  uint16_t attenuator = 1;
  uint16_t speedPercentage = 100;
  unsigned long attenuatorMillis = 0;
};

SoundControllerState sndState;

// Calculate sample count for arrays
template <typename T> size_t array_size(const T &arr) {
  return sizeof(arr) / sizeof(arr[0]);
}

// Generate one stereo sample
void generate_sample(int16_t *left, int16_t *right) {
  // Fixed-rate sounds (group a and b)
  int32_t a1 = 0;
  int32_t b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0, b7 = 0, b8 = 0, b9 = 0, b10 = 0;

  // Horn
  if (sndState.horn || sndState.hornLatch) {
    size_t hornSize = sizeof(horn_samples) / sizeof(horn_samples[0]);
    if (sndState.curHornSample < hornSize - 1) {
      int16_t hornSmaple = horn_samples[sndState.curHornSample];
      a1 = (hornSmaple * vehicle.hornVolume) / 100;
      sndState.curHornSample++;
      if (sndState.horn && sndState.curHornSample == horn_loop_end) {
        sndState.curHornSample = horn_loop_begin;
      }
    } else {
      sndState.curHornSample = 0;
      a1 = 0;
      sndState.hornLatch = false;
    }
  }

  // Reversing beep
  if (modelState.isMovingBackward()) {
    size_t revSize = sizeof(reverse_samples) / sizeof(reverse_samples[0]);
    if (sndState.curReversingSample < revSize - 1) {
      int16_t reverseSample = reverse_samples[sndState.curReversingSample];
      b1 = (reverseSample * vehicle.reversingVolume) / 100;
      sndState.curReversingSample++;
    } else {
      sndState.curReversingSample = 0;
    }
  } else {
    b1 = 0;
    sndState.curReversingSample = 0;
  }

  // Indicator
  if (sndState.turnSignal) {
    size_t indSize = sizeof(blinker_samples) / sizeof(blinker_samples[0]);
    if (sndState.curIndicatorSample < indSize - 1) {
      int16_t indicatorSample = blinker_samples[sndState.curIndicatorSample];
      b2 = (indicatorSample * vehicle.indicatorVolume) / 100;
      sndState.curIndicatorSample++;
    } else {
      sndState.turnSignal = false;
    }
  } else {
    b2 = 0;
    sndState.curIndicatorSample = 0;
  }

  // Air brake release sound "b4", triggered after stop
  if (modelState.isAirBrake()) {
    size_t abLen = sizeof(air_brake_samples) / sizeof(air_brake_samples[0]);
    if (sndState.curBrakeSample < abLen - 1) {
      b4 = (air_brake_samples[sndState.curBrakeSample] * vehicle.brakeVolume / 100);
      sndState.curBrakeSample++;
    } else {
      modelState.setAirBrake(false);
    }
  } else {
    b4 = 0;
    sndState.curBrakeSample = 0;
  }

  // Pneumatic gear shifting sound "b6", triggered while shifting the TAMIYA 3 speed transmission
  if (sndState.shifting && modelState.isEngineRunning()) {
    size_t gsLen = sizeof(gear_shift_samples) / sizeof(gear_shift_samples[0]);
    if (sndState.curShiftingSample < gsLen - 1) {
      b6 = (gear_shift_samples[sndState.curShiftingSample] * vehicle.shiftingVolume / 100);
      sndState.curShiftingSample++;
    } else {
      sndState.shifting = false;
    }
  } else {
    b6 = 0;
    sndState.curShiftingSample = 0; // ensure, next sound will start @ first sample
  }

  // Engine sound resampling
  int16_t leftChannel = 0;
  int16_t rightChannel = 0;

  EngineState engine_state = modelState.getEngineState();

  if (engine_state == STARTING) {
    int16_t startSample = 0;
    if (sndState.phaseStart < engine_start_samples_length - 1) {
      startSample = engine_start_samples[sndState.phaseStart];
      sndState.phaseStart++;
    } else {
      modelState.setEngineState(RUNNING);
      modelState.setAirBrake(true);
      sndState.phaseStart = 0;
    }

    int32_t startScaled = startSample;
    startScaled = (startScaled * sndState.throttleDependentVolume) / 100;
    startScaled = (startScaled * vehicle.startVolume) / 100;
    startScaled = (startScaled * config.volume) / 100;
    leftChannel = constrain(startScaled, -32768, 32767);
  } else if (engine_state == RUNNING) {
    uint32_t ticks = sndState.engineSampleRate ? sndState.engineSampleRate : DEFAULT_TICKS;
    uint32_t phaseInc = (uint32_t)(((uint64_t)TIMER_RESOLUTION_HZ << 16) /
                                   ((uint64_t)ticks * (uint64_t)AUDIO_RATE));

    int32_t engineMixed = 0;

    if (!modelState.isJakeBrake()) {
      // Idle
      sndState.phaseIdle += phaseInc;
      uint32_t maxPhaseIdle = engine_idle_samples_length << 16;
      if (sndState.phaseIdle >= maxPhaseIdle) {
        sndState.phaseIdle -= maxPhaseIdle;
      }

      int32_t idleInterp = 0;
      if (engine_idle_samples_length > 1) {
        uint32_t i = (sndState.phaseIdle >> 16) % engine_idle_samples_length;
        uint32_t j = (i + 1) % engine_idle_samples_length;
        uint32_t frac = sndState.phaseIdle & 0xFFFF;
        int16_t s0 = engine_idle_samples[i];
        int16_t s1 = engine_idle_samples[j];
        idleInterp = s0 + ((int32_t)(s1 - s0) * (int32_t)frac >> 16);

        // Diesel knock triggering
        uint32_t knockInterval = (engine_idle_samples_length << 16) / vehicle.dieselKnockInterval;
        uint32_t phaseDiff = (sndState.phaseIdle >= sndState.lastKnockPhase)
                                 ? (sndState.phaseIdle - sndState.lastKnockPhase)
                                 : (maxPhaseIdle - sndState.lastKnockPhase + sndState.phaseIdle);
        if (phaseDiff >= knockInterval) {
          sndState.dieselKnockTrigger = true;
          sndState.dieselKnockTriggerFirst = (sndState.lastKnockPhase == 0);
          sndState.lastKnockPhase = sndState.phaseIdle;
        }
      }

      // Rev sound
      int32_t revInterp = 0;
      if (vehicle.revSoundEnabled) {
        sndState.phaseRev += phaseInc;
        uint32_t maxPhaseRev = engine_rev_samples_length << 16;
        if (sndState.phaseRev >= maxPhaseRev) {
          sndState.phaseRev -= maxPhaseRev;
        }

        if (engine_rev_samples_length > 1) {
          uint32_t i2 = (sndState.phaseRev >> 16) % engine_rev_samples_length;
          uint32_t j2 = (i2 + 1) % engine_rev_samples_length;
          uint32_t frac2 = sndState.phaseRev & 0xFFFF;
          int16_t r0 = engine_rev_samples[i2];
          int16_t r1 = engine_rev_samples[j2];
          revInterp = r0 + ((int32_t)(r1 - r0) * (int32_t)frac2 >> 16);
        }
      }

      // RPM-based mixing
      uint16_t rpm = modelState.getRPM();
      uint8_t a1Multi;
      if (rpm > vehicle.revSwitchPoint) {
        a1Multi =
            map(rpm, vehicle.idleEndPoint, vehicle.revSwitchPoint, 0, vehicle.idleVolumeProportion);
      } else {
        a1Multi = vehicle.idleVolumeProportion;
      }
      if (rpm > vehicle.idleEndPoint) {
        a1Multi = 0;
      }

      int32_t idleScaled = idleInterp;
      idleScaled = (idleScaled * sndState.throttleDependentVolume) / 100;
      idleScaled = (idleScaled * vehicle.idleVolume) / 100;
      idleScaled = (idleScaled * a1Multi) / 100;

      int32_t revScaled = revInterp;
      revScaled = (revScaled * sndState.throttleDependentRevVolume) / 100;
      revScaled = (revScaled * vehicle.revVolume) / 100;
      revScaled = (revScaled * (100 - a1Multi)) / 100;

      engineMixed = idleScaled + revScaled;
    } else if (vehicle.jakeBrakeEnabled) {
      sndState.phaseJake += phaseInc;
      int32_t jakeInterp = 0;
      if (jake_brake_samples_length > 1) {
        uint32_t i3 = (sndState.phaseJake >> 16) % jake_brake_samples_length;
        if (i3 >= jake_brake_samples_length - 1) {
          sndState.phaseJake = 0;
        }
        uint32_t j3 = (i3 + 1) % jake_brake_samples_length;
        uint32_t frac3 = sndState.phaseJake & 0xFFFF;
        int16_t jb0 = jake_brake_samples[i3];
        int16_t jb1 = jake_brake_samples[j3];
        jakeInterp = jb0 + ((static_cast<int32_t>(jb1 - jb0) * frac3) >> 16);
      }
      engineMixed = jakeInterp;
      engineMixed = (engineMixed * sndState.rpmDependentJakeBrakeVolume) / 100;
      engineMixed = (engineMixed * vehicle.jakeBrakeVolume) / 100;
    }

    // Turbo, fan, charger (simplified - disabled for now)
    int32_t turboRaw = 0, fanRaw = 0, chargerRaw = 0;

    int32_t mixed = ((engineMixed * 8 / 10) + (turboRaw / 5) + (fanRaw / 5) + (chargerRaw / 5));
    int32_t totalMixed = (mixed * config.volume) / 100;
    leftChannel = constrain(totalMixed, -32768, 32767);

    if (!sndState.ignition) {
      modelState.setEngineState(STOPPING);
    }
  } else if (engine_state == STOPPING) {
    int16_t stopSample = 0;
    if (sndState.phaseStop < engine_stop_samples_length - 1) {
      stopSample = engine_stop_samples[sndState.phaseStop];
      sndState.phaseStop++;
    } else {
      modelState.setEngineState(PARKING_BRAKE);
      sndState.phaseStop = 0;
    }

    int32_t stoppingScaled = stopSample;
    stoppingScaled = (stoppingScaled * config.volume) / 100;
    leftChannel = constrain(stoppingScaled, -32768, 32767);
  } else if (engine_state == PARKING_BRAKE) {
    if (sndState.curParkingBrakeSample < parking_brake_samples_length - 1) {
      leftChannel = (parking_brake_samples[sndState.curParkingBrakeSample] *
                     vehicle.parkingBrakeVolume / 100);
      sndState.curParkingBrakeSample++;
    } else {
      sndState.curParkingBrakeSample = 0;
      leftChannel = 0;
      modelState.setEngineState(OFF);
    }
  } else {
    // Engine OFF or unknown state
    leftChannel = 0;
  }

  // Right channel (group a and b sounds)
  int32_t groupA = (a1 * 8 / 10);
  int32_t groupB = (b1 + b2 + b3 + b4 + b5 + b6 + b7 + b8 + b9 + b10) / 2;
  int32_t rightMixed = ((groupA + groupB) * config.volume) / 100;
  rightChannel = constrain(rightMixed, -32768, 32767);

  *left = leftChannel;
  *right = rightChannel;
}

// PortAudio callback
static int audio_callback(const void *inputBuffer, void *outputBuffer,
                          unsigned long framesPerBuffer, const PaStreamCallbackTimeInfo *timeInfo,
                          PaStreamCallbackFlags statusFlags, void *userData) {
  int16_t *out = static_cast<int16_t *>(outputBuffer);
  (void)inputBuffer;
  (void)timeInfo;
  (void)statusFlags;
  (void)userData;

  for (unsigned long i = 0; i < framesPerBuffer; i++) {
    int16_t left, right;
    generate_sample(&left, &right);
    *out++ = left;
    *out++ = right;
  }

  return paContinue;
}

// Update sound controller state
void onRpmChange() {
  uint16_t value = modelState.getRPM();
  sndState.engineSampleRate =
      map(value, MIN_RPM, MAX_RPM, sndState.maxSampleInterval, sndState.minSampleInterval);

  if (vehicle.dieselKnockDependsOnRPM) {
    if (value > 400) {
      sndState.rpmDependentKnockVolume = map(value, 400, MAX_RPM, 5, 100);
    } else {
      sndState.rpmDependentKnockVolume = 5;
    }
  }

  if (vehicle.jakeBrakeEnabled) {
    if (modelState.isEngineRunning()) {
      sndState.rpmDependentJakeBrakeVolume =
          map(value, MIN_RPM, MAX_RPM, vehicle.jakeBrakeIdleVolume, 100);
    } else {
      sndState.rpmDependentJakeBrakeVolume = vehicle.jakeBrakeIdleVolume;
    }
  }

  if (vehicle.turboEnabled) {
    if (modelState.isEngineRunning()) {
      sndState.throttleDependentTurboVolume =
          map(value, MIN_RPM, MAX_RPM, vehicle.turboIdleVolume, 100);
    } else {
      sndState.throttleDependentTurboVolume = vehicle.turboIdleVolume;
    }
  }

  if (vehicle.wastegateEnabled) {
    if (modelState.isEngineRunning()) {
      sndState.rpmDependentWastegateVolume =
          map(value, MIN_RPM, MAX_RPM, vehicle.wastegateIdleVolume, 100);
    } else {
      sndState.rpmDependentWastegateVolume = vehicle.wastegateIdleVolume;
    }
  }
}

void onThrottleChange(uint16_t value) {
  // Fade throttle
  if (sndState.currentThrottleFaded < value && sndState.currentThrottleFaded < 499) {
    sndState.currentThrottleFaded += 32;
    if (sndState.currentThrottleFaded > value) {
      sndState.currentThrottleFaded = value;
    }
  } else if (sndState.currentThrottleFaded > value && sndState.currentThrottleFaded > 32) {
    sndState.currentThrottleFaded -= 32;
    if (sndState.currentThrottleFaded < value) {
      sndState.currentThrottleFaded = value;
    }
  }

  // Calculate throttle dependent volumes
  if (modelState.isEngineRunningNotBreaking()) {
    sndState.throttleDependentVolume = map(sndState.currentThrottleFaded, 0, 500,
                                           vehicle.engineIdleVolume, vehicle.fullThrottleVolume);
  } else {
    sndState.throttleDependentVolume = vehicle.engineIdleVolume;
  }

  if (vehicle.revSoundEnabled) {
    if (modelState.isEngineRunningNotBreaking()) {
      sndState.throttleDependentRevVolume =
          map(sndState.currentThrottleFaded, 0, 500, vehicle.engineRevVolume,
              vehicle.fullThrottleVolume);
    } else {
      sndState.throttleDependentRevVolume = vehicle.engineRevVolume;
    }
  }

  if (modelState.isEngineRunningNotBreaking() &&
      sndState.currentThrottleFaded > vehicle.dieselKnockStartPoint) {
    sndState.throttleDependentKnockVolume =
        map(sndState.currentThrottleFaded, vehicle.dieselKnockStartPoint, 500,
            vehicle.dieselKnockIdleVolume, 100);
  } else {
    sndState.throttleDependentKnockVolume = vehicle.dieselKnockIdleVolume;
  }
}

// Global flag for ncurses update
std::atomic<bool> ncurses_initialized(false);
WINDOW *status_win = nullptr;

// Update ncurses display
void update_display() {
  if (!ncurses_initialized || !status_win)
    return;

  werase(status_win);
  box(status_win, 0, 0);

  int row = 1;
  mvwprintw(status_win, row++, 2, "=== Sound Simulator Status ===");
  row++;

  // Engine state
  const char *engineStateStr = "OFF";
  switch (modelState.getEngineState()) {
  case OFF:
    engineStateStr = "OFF";
    break;
  case STARTING:
    engineStateStr = "STARTING";
    break;
  case RUNNING:
    engineStateStr = "RUNNING";
    break;
  case STOPPING:
    engineStateStr = "STOPPING";
    break;
  case PARKING_BRAKE:
    engineStateStr = "PARKING_BRAKE";
    break;
  }
  mvwprintw(status_win, row++, 2, "Engine State: %s", engineStateStr);

  // Ignition
  mvwprintw(status_win, row++, 2, "Ignition: %s", sndState.ignition ? "ON " : "OFF");

  // RPM
  mvwprintw(status_win, row++, 2, "RPM: %4d", modelState.getRPM());

  // Throttle
  mvwprintw(status_win, row++, 2, "Throttle: %3d", modelState.getThrottle());

  // Controls
  mvwprintw(status_win, row++, 2, "Horn: %s", sndState.horn ? "ON " : "OFF");
  mvwprintw(status_win, row++, 2, "Brake: %s", modelState.isBraking() ? "ON " : "OFF");
  mvwprintw(status_win, row++, 2, "Jake Brake: %s", modelState.isJakeBrake() ? "ON " : "OFF");
  mvwprintw(status_win, row++, 2, "Turn Signal: %s",
            modelState.getTurnSignal() != T_OFF ? "ON " : "OFF");
  mvwprintw(status_win, row++, 2, "Wastegate: %s", sndState.wastegate ? "TRIGGERED" : "OFF");

  row++;
  mvwprintw(status_win, row++, 2, "================================");
  row++;
  mvwprintw(status_win, row++, 2, "Controls:");
  mvwprintw(status_win, row++, 2, "  i - Ignition on/off");
  mvwprintw(status_win, row++, 2, "  + - Increase throttle");
  mvwprintw(status_win, row++, 2, "  - - Decrease throttle");
  mvwprintw(status_win, row++, 2, "  h - Horn");
  mvwprintw(status_win, row++, 2, "  b - Brake");
  mvwprintw(status_win, row++, 2, "  j - Jake brake");
  mvwprintw(status_win, row++, 2, "  s - Turn signal");
  mvwprintw(status_win, row++, 2, "  1 - Gear 1");
  mvwprintw(status_win, row++, 2, "  2 - Gear 2");
  mvwprintw(status_win, row++, 2, "  3 - Gear 3");
  mvwprintw(status_win, row++, 2, "  q - Quit");

  wrefresh(status_win);
}

// Keyboard input handler with ncurses
void keyboard_control() {
  // Initialize ncurses
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);

  // Create status window
  int height = 28;
  int width = 40;
  int starty = (LINES - height) / 2;
  int startx = (COLS - width) / 2;

  status_win = newwin(height, width, starty, startx);
  if (!status_win) {
    endwin();
    return;
  }

  ncurses_initialized = true;

  std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
  int ch;
  while (true) {
    ch = getch();

    if (ch == 'q' || ch == 'Q') {
      break;
    } else if (ch == 'i' || ch == 'I') {
      sndState.ignition = !sndState.ignition;
      if (sndState.ignition && modelState.getEngineState() == OFF) {
        modelState.setEngineState(STARTING);
      }
    } else if (ch == '+') {
      int16_t throttle = modelState.getThrottle() + 5;
      if (throttle > 500) {
        throttle = 500;
      }
      modelState.setThrottle(throttle);
      onThrottleChange(modelState.getThrottle());

      int16_t rpm = modelState.getRPM() + 5;
      if (rpm > 500) {
        rpm = 500;
      }
      modelState.setRPM(rpm);
      onRpmChange();
    } else if (ch == '-') {
      int16_t throttle = modelState.getThrottle() - 5;
      if (throttle < 0) {
        throttle = 0;
      }
      modelState.setThrottle(throttle);
      onThrottleChange(modelState.getThrottle());

      int16_t rpm = modelState.getRPM() - 5;
      if (rpm < 0) {
        rpm = 0;
      }
      modelState.setRPM(rpm);
      onRpmChange();
    } else if (ch == 'h' || ch == 'H') {
      sndState.horn = !sndState.horn;
      if (sndState.horn) {
        sndState.hornLatch = true;
      }
    } else if (ch == 'b' || ch == 'B') {
      bool br = modelState.isBraking();
      if (br) {
        modelState.setDriveState(DRIVING_FORWARD);
      } else {
        modelState.setDriveState(BRAKING_FORWARD);
      }
    } else if (ch == 'j' || ch == 'J') {
      bool jb = modelState.isJakeBrake();
      modelState.setJakeBrake(!jb);
    } else if (ch == 's' || ch == 'S') {
      TurnSignal ts = modelState.getTurnSignal();
      ts = (ts == T_OFF) ? T_LEFT : T_OFF;
      modelState.setTurnSignal(ts);
    } else if (ch == 'w' || ch == 'W') {
      sndState.wastegate = true;
    } else if (ch == '1' || ch == '2' || ch == '3') {
      sndState.shifting = true;
    }

    // Update display
    update_display();
    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
    if (modelState.getTurnSignal() != T_OFF) {
      if (std::chrono::duration_cast<std::chrono::microseconds>(now - begin).count() >= 375000) {
        sndState.turnSignal = true;
        begin = now;
      }
    } else {
      sndState.turnSignal = false;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  // Cleanup ncurses
  if (status_win) {
    delwin(status_win);
  }
  endwin();
  ncurses_initialized = false;
}

int main() {
  // Initialize sound controller state
  sndState.minSampleInterval = DEFAULT_TICKS * 100 / vehicle.maxRpmPercentage;
  sndState.maxSampleInterval = DEFAULT_TICKS;
  sndState.engineSampleRate = sndState.maxSampleInterval;

  // Initialize PortAudio
  PaError err = Pa_Initialize();
  if (err != paNoError) {
    std::cerr << "PortAudio initialization failed: " << Pa_GetErrorText(err) << std::endl;
    return 1;
  }

  PaStream *stream;
  err = Pa_OpenDefaultStream(&stream, 0, 2, paInt16, AUDIO_RATE, 256, audio_callback, nullptr);
  if (err != paNoError) {
    std::cerr << "PortAudio stream open failed: " << Pa_GetErrorText(err) << std::endl;
    Pa_Terminate();
    return 1;
  }

  err = Pa_StartStream(stream);
  if (err != paNoError) {
    std::cerr << "PortAudio stream start failed: " << Pa_GetErrorText(err) << std::endl;
    Pa_CloseStream(stream);
    Pa_Terminate();
    return 1;
  }

  // Start keyboard control in separate thread (ncurses will initialize)
  std::thread kb_thread(keyboard_control);

  // Give ncurses time to initialize
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Initial display update
  if (ncurses_initialized) {
    update_display();
  }

  // Wait for quit
  kb_thread.join();

  // Cleanup
  Pa_StopStream(stream);
  Pa_CloseStream(stream);
  Pa_Terminate();

  return 0;
}
