#pragma once

#include "signal.hpp"

class Timer {
public:
  // signal that will be emitted when the alarm triggers.
  sigslot::signal<> onAlarm;

  virtual ~Timer() {
  }

  virtual void start() = 0;
  virtual void stop() = 0;
  virtual bool is_running() = 0;

protected:
  static inline const char *TAG = "TMR";
};
