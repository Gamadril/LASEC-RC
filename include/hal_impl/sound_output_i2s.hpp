#pragma once

#include "../hal/sound_output.hpp"

#include "board_config.hpp"
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// I2S configuration - larger DMA buffers for queue-based approach
#define I2S_DMA_FRAME_COUNT 256
#define I2S_DMA_DESC_COUNT 8
#define AUDIO_QUEUE_LENGTH 32 // Queue depth for ISR->task communication

class SoundOutputI2S : public SoundOutput {
public:
  void init(uint16_t audio_rate) override {
    esp_err_t err;
    /* I2S init */
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    // Low latency configuration: 3 buffers of 128 frames (~8ms latency at 44.1kHz)
    chan_cfg.dma_desc_num = 3;
    chan_cfg.dma_frame_num = 128;
    chan_cfg.auto_clear = true;

    err = i2s_new_channel(&chan_cfg, &_i2s_tx_chan, nullptr);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "i2s channel create failed: %d", err);
      return;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(audio_rate),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg =
            {
                .mclk = I2S_GPIO_UNUSED,
                .bclk = static_cast<gpio_num_t>(PIN_I2S_BCLK),
                .ws = static_cast<gpio_num_t>(PIN_I2S_LRC),
                .dout = static_cast<gpio_num_t>(PIN_I2S_DOUT),
                .din = I2S_GPIO_UNUSED,
                .invert_flags =
                    {
                        .mclk_inv = false,
                        .bclk_inv = false,
                        .ws_inv = false,
                    },
            },
    };
    std_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;

    err = i2s_channel_init_std_mode(_i2s_tx_chan, &std_cfg);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "i2s std mode init failed: %d", err);
      return;
    }

    err = i2s_channel_enable(_i2s_tx_chan);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "i2s channel enable failed: %d", err);
      return;
    }

    // Create high-priority I2S write task pinned to core 1 (app core)
    BaseType_t taskCreated = xTaskCreatePinnedToCore(&_i2s_write_task,         // Task function
                                                     "I2S_Write",              // Task name
                                                     4096,                     // Stack size
                                                     this,                     // Parameters
                                                     configMAX_PRIORITIES - 1, // High priority
                                                     &_i2sWriteTaskHandle,     // Task handle
                                                     1 // Core 1 (app core, core 0 runs protocol stack)
    );

    if (taskCreated != pdPASS) {
      ESP_LOGE(TAG, "Failed to create I2S write task");
      return;
    }
  }

  void deinit() override {
    if (_i2sWriteTaskHandle) {
      vTaskDelete(_i2sWriteTaskHandle);
      _i2sWriteTaskHandle = nullptr;
    }

    if (_i2s_tx_chan) {
      i2s_channel_disable(_i2s_tx_chan);
      i2s_del_channel(_i2s_tx_chan);
      _i2s_tx_chan = nullptr;
    }
  }

private:
  static inline const char *TAG = "SND";
  i2s_chan_handle_t _i2s_tx_chan;
  TaskHandle_t _i2sWriteTaskHandle;

  // I2S write task - runs continuously, generates samples and writes to I2S
  static void _i2s_write_task(void *arg) {
    SoundOutputI2S *instance = static_cast<SoundOutputI2S *>(arg);

    if (instance) {
      const size_t CHUNK_SIZE = 64; // Number of samples to generate per write
      AudioSample samples[CHUNK_SIZE];
      int16_t stereoBuffer[CHUNK_SIZE * 2]; // Interleaved stereo buffer
      size_t bytesWritten;

      ESP_LOGI(TAG, "I2S write task started on core %d", xPortGetCoreID());

      while (true) {
        // Generate a chunk of samples
        for (size_t i = 0; i < CHUNK_SIZE; i++) {
          instance->_get_sample_func(&samples[i], instance->_callback_user_data);
          stereoBuffer[i * 2] = samples[i].left;
          stereoBuffer[i * 2 + 1] = samples[i].right;
        }

        // Blocking write to I2S - DMA pacing handles timing
        // No logging in the loop to prevent starvation/latency
        i2s_channel_write(instance->_i2s_tx_chan, stereoBuffer, sizeof(stereoBuffer), &bytesWritten, portMAX_DELAY);
      }
    }

    vTaskDelete(NULL);
  }
};