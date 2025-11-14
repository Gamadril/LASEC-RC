#pragma once

#include "board_config.h"
#include <FastLED.h>

class RgbController {
public:
  RgbController() {
  }

  void init(uint8_t ledCount) {
    _ledCount = ledCount;
    _leds = new CRGB[_ledCount];
    FastLED.addLeds<NEOPIXEL, PIN_RGB_LED>(_leds, _ledCount);
  }

  void show() {
    FastLED.show();
  }

  void setBrightness(uint8_t brightness) {
    FastLED.setBrightness(brightness);
  }

  uint8_t getBrightness() {
    return FastLED.getBrightness();
  }

  void setColour(uint8_t index, const struct CRGB &colour) {
    _leds[index] = colour;
  }

  void setColour(uint8_t index, const struct CHSV &colour) {
    _leds[index] = colour;
  }

  void setColour(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    _leds[index].setRGB(r, g, b);
  }

  void setColour(uint8_t index, uint32_t colour_code) {
    _leds[index] = colour_code;
  }

  CRGB &getColour(uint8_t index) {
    return _leds[index];
  }

private:
  uint8_t _ledCount;
  CRGB *_leds;
};
