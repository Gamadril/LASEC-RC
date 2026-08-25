#pragma once

#include "board_config.hpp"
#if defined(ESP_PLATFORM)
#include "led_strip.h"
#endif

// Simple RGB colour container (replaces FastLED's CRGB)
struct RgbColour {
  uint8_t r = 0;
  uint8_t g = 0;
  uint8_t b = 0;
};

// Simple HSV colour container (replaces FastLED's CHSV)
// hue/saturation/value all use the 0-255 range, same convention as CHSV
struct RgbHsv {
  uint8_t h = 0;
  uint8_t s = 0;
  uint8_t v = 0;
};

class RgbController {
public:
  RgbController() = default;

  ~RgbController() {
#if defined(ESP_PLATFORM)
    if (_strip != nullptr) {
      led_strip_del(_strip);
    }
#endif
    delete[] _leds;
  }

  /**
   * Initialize the LED strip
   * @param ledCount Number of LEDs on the strip
   */
  void init(uint8_t ledCount) {
    _ledCount = ledCount;
    _leds = new RgbColour[_ledCount];

#if defined(ESP_PLATFORM)
    led_strip_config_t strip_config = {};
    strip_config.strip_gpio_num = PIN_RGB_LED;
    strip_config.max_leds = _ledCount;
    strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
    strip_config.led_model = LED_MODEL_WS2812;
    strip_config.flags.invert_out = false;

    led_strip_rmt_config_t rmt_config = {};
    rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
    rmt_config.resolution_hz = 10 * 1000 * 1000; // 10MHz
    rmt_config.flags.with_dma = false;

    esp_err_t err = led_strip_new_rmt_device(&strip_config, &rmt_config, &_strip);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize LED strip: %s", esp_err_to_name(err));
      _strip = nullptr;
      return;
    }

    led_strip_clear(_strip);
#endif
  }

  /**
   * Push the current colour buffer (with brightness applied) out to the strip
   */
  void show() {
#if defined(ESP_PLATFORM)
    if (_strip == nullptr) {
      return;
    }

    for (uint8_t i = 0; i < _ledCount; i++) {
      const RgbColour &c = _leds[i];
      uint8_t scaledR = (static_cast<uint16_t>(c.r) * _brightness) / 255;
      uint8_t scaledG = (static_cast<uint16_t>(c.g) * _brightness) / 255;
      uint8_t scaledB = (static_cast<uint16_t>(c.b) * _brightness) / 255;
      led_strip_set_pixel(_strip, i, scaledR, scaledG, scaledB);
    }

    esp_err_t err = led_strip_refresh(_strip);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to refresh LED strip: %s", esp_err_to_name(err));
    }
#endif
  }

  /**
   * Set global brightness (0-255), applied on the next show()
   */
  void setBrightness(uint8_t brightness) { _brightness = brightness; }

  uint8_t getBrightness() { return _brightness; }

  void setColour(uint8_t index, const RgbColour &colour) { _leds[index] = colour; }

  void setColour(uint8_t index, const RgbHsv &colour) { _leds[index] = hsv2rgb(colour); }

  void setColour(uint8_t index, uint8_t r, uint8_t g, uint8_t b) {
    _leds[index].r = r;
    _leds[index].g = g;
    _leds[index].b = b;
  }

  void setColour(uint8_t index, uint32_t colour_code) {
    _leds[index].r = (colour_code >> 16) & 0xFF;
    _leds[index].g = (colour_code >> 8) & 0xFF;
    _leds[index].b = colour_code & 0xFF;
  }

  /** Pack LED colour as 0x00RRGGBB */
  uint32_t getColourCode(uint8_t index) const {
    const RgbColour &c = _leds[index];
    return (static_cast<uint32_t>(c.r) << 16) | (static_cast<uint32_t>(c.g) << 8) | c.b;
  }

  RgbColour &getColour(uint8_t index) { return _leds[index]; }

private:
  static inline const char *TAG = "RGB";

  /**
   * Standard HSV (0-255 range) to RGB conversion.
   * Note: this is a generic HSV->RGB conversion, not FastLED's rainbow-based hue mapping,
   * so exact hue positions may differ slightly from FastLED's CHSV output.
   */
  static RgbColour hsv2rgb(const RgbHsv &hsv) {
    uint8_t region = hsv.h / 43;
    uint8_t remainder = (hsv.h - (region * 43)) * 6;

    uint8_t p = (hsv.v * (255 - hsv.s)) >> 8;
    uint8_t q = (hsv.v * (255 - ((hsv.s * remainder) >> 8))) >> 8;
    uint8_t t = (hsv.v * (255 - ((hsv.s * (255 - remainder)) >> 8))) >> 8;

    switch (region) {
    case 0:
      return {hsv.v, t, p};
    case 1:
      return {q, hsv.v, p};
    case 2:
      return {p, hsv.v, t};
    case 3:
      return {p, q, hsv.v};
    case 4:
      return {t, p, hsv.v};
    default:
      return {hsv.v, p, q};
    }
  }

  uint8_t _ledCount = 0;
  uint8_t _brightness = 255;
  RgbColour *_leds = nullptr;

#if defined(ESP_PLATFORM)
  led_strip_handle_t _strip = nullptr;
#endif
};