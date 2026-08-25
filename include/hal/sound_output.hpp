#pragma once

#include "../utils.hpp"
#include <cstdint>

// Raw function pointer callback - safe for IRAM
typedef void (*SampleCallback)(AudioSample *frame, void *user_data);

class SoundOutput {
public:
  virtual ~SoundOutput() {
  }

  virtual void init(uint16_t audio_rate) = 0;
  virtual void deinit() = 0;
  void setSampleCallback(SampleCallback callback, void *user_data) {
    _get_sample_func = callback;
    _callback_user_data = user_data;
  }

protected:
  SampleCallback _get_sample_func;
  void *_callback_user_data;
};