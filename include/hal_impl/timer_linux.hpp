#pragma once

#include "../hal/timer.hpp"
#include "signal.hpp"
#include <sys/timerfd.h>
#include <thread>

class TimerLinux : public Timer {
public:
  sigslot::signal<> onAlarm;

  TimerLinux(uint32_t period_ms) : _period_ms(period_ms) {
    _fd = timerfd_create(CLOCK_MONOTONIC, 0);

    struct itimerspec spec = {};
    spec.it_interval.tv_sec = 0;
    spec.it_interval.tv_nsec = _period_ms * 1000000LL;

    spec.it_value = spec.it_interval; // first expiration

    timerfd_settime(_fd, 0, &spec, NULL);
  }

  ~TimerLinux() {
    _running = false;
    close(_fd);
    if (_thread.joinable())
      _thread.join();
  }

  void start() override {
    _running = true;
    _thread = std::thread([this] {
      uint64_t expirations;
      while (_running) {
        read(_fd, &expirations, sizeof(expirations));
        onAlarm();
      }
    });
  }

  void stop() override {
    _running = false;
    if (_thread.joinable())
      _thread.join();
  }

  bool is_running() override {
    return _running;
  }

private:
  bool _running = false;
  uint32_t _period_ms;
  int _fd;
  std::thread _thread;
};
