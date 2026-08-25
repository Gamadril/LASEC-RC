#pragma once

#include "../hal/wav_reader.hpp"
#include <cstdint>
#include <cstdio>
#include <string>

#include <esp_err.h>
#include <esp_log.h>
#include <esp_partition.h>
#include <spi_flash_mmap.h>

// Static mmap handle shared by all instances
const void *mmap_ptr;
spi_flash_mmap_handle_t mmap_handle;

class WavReaderMMap : public WavReader {
public:
  void init(const std::string &file_path, uint32_t loop_start = 0, uint32_t loop_end = 0) override {
    _loop_start = loop_start;
    _loop_end = loop_end;

    if (!mmap_ptr) {
      _init_mmap();
    }

    // We need to read 8 bytes: offset (4), length (4)
    FILE *f = fopen(file_path.c_str(), "rb");
    if (!f) {
      ESP_LOGE(TAG, "Meta file not found: %s", file_path.c_str());
      return;
    }

    uint32_t offset = 0;
    uint32_t length = 0;
    fread(&offset, 4, 1, f);
    fread(&length, 4, 1, f);
    fclose(f);

    _data_start = (const uint8_t *)mmap_ptr + offset;
    _data_len = length;

    const uint8_t *pos = _data_start;
    const uint8_t *end = _data_start + _data_len;

    // Read WAV header
    WAVHeader header;
    memcpy(&header, pos, sizeof(WAVHeader));
    pos += sizeof(WAVHeader);

    // Validate RIFF header
    if (std::strncmp(header.riff, "RIFF", 4) != 0 || std::strncmp(header.wave, "WAVE", 4) != 0) {
      ESP_LOGE(TAG, "Error: Invalid WAV file format: %s", file_path.c_str());
      return;
    }

    // Validate size
    if (header.chunkSize + 8 != length) {
      ESP_LOGE(TAG, "Error: Invalid size. header: %d, meta: %d. %s", header.chunkSize + 8, length, file_path.c_str());
      return;
    }

    // Find and read fmt chunk
    WAVFmtChunk fmtChunk;
    memcpy(&fmtChunk, pos, sizeof(WAVFmtChunk));
    pos += sizeof(WAVFmtChunk);
    if (std::strncmp(fmtChunk.fmt, "fmt ", 4) != 0) {
      ESP_LOGE(TAG, "Error: Invalid fmt chunk: %s", file_path.c_str());
      return;
    }

    // Validate format
    if (fmtChunk.audioFormat != 1) {
      ESP_LOGE(TAG, "Error: Only PCM format is supported (found format: %d): %s", fmtChunk.audioFormat,
               file_path.c_str());
      return;
    }

    if (fmtChunk.sampleRate != 44100) {
      ESP_LOGE(TAG, "Error: Sample rate must be 44100 Hz (found %d Hz): %s", fmtChunk.sampleRate, file_path.c_str());
      return;
    }

    if (fmtChunk.bitsPerSample != 16) {
      ESP_LOGE(TAG, "Error: Bit depth must be 16-bit (found %d-bit): %s", fmtChunk.bitsPerSample, file_path.c_str());
      return;
    }

    if (fmtChunk.numChannels != 1) {
      ESP_LOGE(TAG, "Error: Only mono channel supported (found %d channels): %s", fmtChunk.numChannels,
               file_path.c_str());
      return;
    }

    // Find data chunk
    WAVDataChunk dataChunk;
    memcpy(&dataChunk, pos, sizeof(WAVDataChunk));
    pos += sizeof(WAVDataChunk);
    if (std::strncmp(dataChunk.data, "data", 4) != 0) {
      ESP_LOGE(TAG, "Error: Invalid data chunk: %s", file_path.c_str());
      return;
    }

    _samplesStartPtr = pos;
    _samplesCount = dataChunk.chunkSize / (fmtChunk.bitsPerSample / 8);

    _cursor = 0;
  }

  void deinit() override {
  }

  size_t get_samples_count() override {
    return _samplesCount;
  }

  int16_t read_sample(uint32_t index) override {
    if (index >= _samplesCount)
      return 0;
    const uint8_t *p = _samplesStartPtr + index * 2;
    _cursor = index + 1;
    return p[0] | (p[1] << 8);
  }

  int16_t read_next_sample(bool is_looping = false) override {
    if (is_looping) {
      if (_cursor >= _loop_end) {
        _cursor = _loop_start;
      }
    } else if (_cursor >= _samplesCount) {
      _cursor = 0;
    }

    const uint8_t *p = _samplesStartPtr + _cursor * 2;
    _cursor++;
    return p[0] | (p[1] << 8);
  }

  void reset() override {
    _cursor = 0;
  }

  bool is_start() override {
    return _cursor == 0;
  }

  bool is_end() override {
    return _cursor >= _samplesCount;
  }

protected:
  void _init_mmap() {
    if (mmap_ptr)
      return;

    const esp_partition_t *part =
        esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_UNDEFINED, "audio_data");
    if (!part) {
      ESP_LOGE(TAG, "Audio partition not found!");
      return;
    }

    esp_err_t err = esp_partition_mmap(part, 0, part->size, ESP_PARTITION_MMAP_DATA, &mmap_ptr, &mmap_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Mmap failed: %s", esp_err_to_name(err));
    } else {
      ESP_LOGI(TAG, "Audio mapped at %p, size: %d", mmap_ptr, part->size);
    }
  }

private:
  static inline const char *TAG = "MMAP";
  const uint8_t *_data_start = nullptr;
  const uint8_t *_samplesStartPtr = nullptr;
  size_t _data_len = 0;
  size_t _samplesCount = 0;
  uint32_t _cursor = 0;
  uint32_t _loop_start = 0;
  uint32_t _loop_end = 0;
  uint8_t _bytesPerSample = 2;
};
