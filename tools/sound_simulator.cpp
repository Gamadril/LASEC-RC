#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>
#include <unistd.h>
#include <vector>

#include <ncurses.h>

#define MIN_RPM 0
#define MAX_RPM 500

#include "config_handler.hpp"
#include "hal_impl/persistence_fs.hpp"
#include "hal_impl/sound_outout_pa.hpp"
#include "hal_impl/timer_linux.hpp"
#include "hal_impl/wav_reader_fs.hpp"
#include "model_state.hpp"
#include "signal_handler.hpp"
#include "sound_controller.hpp"
#include "sound_manager.hpp"

// Global flag for ncurses update
std::atomic<bool> ncurses_initialized(false);
WINDOW *status_win = nullptr;

PersistenceFS persistence;
ConfigHandler configHandler(persistence, "config.json");
ModelState state;
EscController escController;
ServoController servoController;
ReceiverController receiverController(nullptr);
SoundOutputPA soundOutput;
SoundManager soundManager([]() { return new WavReaderFS(); });
SoundController *soundController = nullptr;
LightsController lightsController([]() { return new TimerLinux(345); });

bool exitMain = false;

// Update ncurses display
void update_display() {
  if (!ncurses_initialized || !status_win)
    return;

  werase(status_win);
  box(status_win, 0, 0);

  int row = 1;
  mvwprintw(status_win, row++, 2, "=== Sound Simulator Status ===");
  row++;

  mvwprintw(status_win, row++, 2, "Drive State: %s", ds2str(state.getDriveState()).c_str());
  mvwprintw(status_win, row++, 2, "Engine State: %s", es2str(state.getEngineState()).c_str());
  mvwprintw(status_win, row++, 2, "RPM: %4d", state.getRPM());
  mvwprintw(status_win, row++, 2, "Throttle: %3d", state.getThrottle());
  mvwprintw(status_win, row++, 2, "Horn: %s", state.isHorn() ? "ON " : "OFF");
  mvwprintw(status_win, row++, 2, "Hazard: %s", state.isHazard() ? "ON " : "OFF");
  mvwprintw(status_win, row++, 2, "Turn Signal: %s", state.getTurnSignal() != T_OFF ? "ON " : "OFF");
  row++;
  mvwprintw(status_win, row++, 2, "================================");
  row++;
  mvwprintw(status_win, row++, 2, "Controls:");
  mvwprintw(status_win, row++, 2, "  w - left stick -> up");
  mvwprintw(status_win, row++, 2, "  s - left stick -> down");
  mvwprintw(status_win, row++, 2, "  l - right stick -> right");
  mvwprintw(status_win, row++, 2, "  j - right stick -> left");
  mvwprintw(status_win, row++, 2, "  c - Clutch");
  mvwprintw(status_win, row++, 2, "  h - Horn");
  mvwprintw(status_win, row++, 2, "  z - Hazard");
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

  int ch;
  uint16_t channels[16];
  for (int i = 0; i < 16; i++) {
    channels[i] = 1500;
  }
  while (true) {
    ch = getch();

    if (ch == 'q' || ch == 'Q') {
      break;
    } else if (ch == 'w') {
      channels[CH_THROTTLE - 1] += 10;
    } else if (ch == 's') {
      channels[CH_THROTTLE - 1] -= 10;
    } else if (ch == 'j') {
      channels[CH_STEERING - 1] += 10;
    } else if (ch == 'l') {
      channels[CH_STEERING - 1] -= 10;
    } else if (ch == 'c') {
      if (channels[CH_GEAR_CLUTCH - 1] == 2000) {
        channels[CH_GEAR_CLUTCH - 1] = 1500;
      } else {
        channels[CH_GEAR_CLUTCH - 1] = 2000;
      }
    } else if (ch == 'h') {
      if (channels[CH_HORN - 1] == 2000) {
        channels[CH_HORN - 1] = 1500;
      } else {
        channels[CH_HORN - 1] = 2000;
      }
    } else if (ch == 'z') {
      if (channels[CH_HAZARD - 1] == 2000) {
        channels[CH_HAZARD - 1] = 1500;
      } else {
        channels[CH_HAZARD - 1] = 2000;
      }
    } else if (ch == '1' || ch == '2' || ch == '3') {
      if (ch == '1') {
        channels[CH_SHIFTING - 1] = 1000;
      } else if (ch == '2') {
        channels[CH_SHIFTING - 1] = 1500;
      } else if (ch == '3') {
        channels[CH_SHIFTING - 1] = 2000;
      }
    }
    receiverController.onFrame(channels, 16, false);

    // Update display
    update_display();

    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }

  exitMain = true;

  // Cleanup ncurses
  if (status_win) {
    delwin(status_win);
  }
  endwin();
  ncurses_initialized = false;
}

int main() {
  configHandler.init();
  Config *config = configHandler.getConfig();
  std::cout << "Config name: " << config->model_name << std::endl;

  soundManager.scan("../sounds");
  std::cout << "Sound assets loaded: " << soundManager.getSounds().size() << std::endl;

  lightsController.init();
  soundController = new SoundController(soundOutput, soundManager.getSounds());
  soundController->init(&state, config);

  escController.init(config);
  servoController.init(config);
  receiverController.init();

  setupSignalHandlers(&state, &lightsController, soundController, &escController, &receiverController, &servoController,
                      config);

  // Start keyboard control in separate thread (ncurses will initialize)
  std::thread kb_thread(keyboard_control);

  // Give ncurses time to initialize
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  std::thread main_thread(
      [configPtr = config, statePtr = &state, soundPtr = soundController, escPtr = &escController]() {
        while (!exitMain) {
          statePtr->checkGearShiftingStop();
          engineMassSimulation(configPtr, statePtr, soundPtr, escPtr);
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
      });

  // Initial display update
  if (ncurses_initialized) {
    update_display();
  }

  // Wait for quit
  kb_thread.join();
  main_thread.join();

  return 0;
}
