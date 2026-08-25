#pragma once

#include "../utils.hpp"
#include <cstdint>
#include <string>

// WAV file structures
struct WAVHeader {
  char riff[4];       // "RIFF"
  uint32_t chunkSize; // File size - 8
  char wave[4];       // "WAVE"
};

struct WAVFmtChunk {
  char fmt[4];            // "fmt "
  uint32_t chunkSize;     // Usually 16 for PCM
  uint16_t audioFormat;   // 1 = PCM
  uint16_t numChannels;   // 1 = mono, 2 = stereo
  uint32_t sampleRate;    // e.g., 44100
  uint32_t byteRate;      // sampleRate * numChannels * bitsPerSample / 8
  uint16_t blockAlign;    // numChannels * bitsPerSample / 8
  uint16_t bitsPerSample; // e.g., 16
};

struct WAVDataChunk {
  char data[4];       // "data"
  uint32_t chunkSize; // Size of audio data
};

class WavReader {
public:
  virtual ~WavReader() {
  }

  virtual void init(const std::string &file_path, uint32_t loop_start = 0, uint32_t loop_end = 0) = 0;
  virtual void deinit() = 0;
  virtual size_t get_samples_count() = 0;
  virtual int16_t read_sample(uint32_t index) = 0;
  virtual int16_t read_next_sample(bool is_looping = false) = 0;
  virtual void reset() = 0;
  virtual bool is_start() = 0;
  virtual bool is_end() = 0;
};