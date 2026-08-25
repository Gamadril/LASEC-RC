#pragma once

#include "../hal/wav_reader.hpp"
#include "../utils.hpp"
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

class WavReaderFS : public WavReader {
public:
  void init(const std::string &file_path, uint32_t loop_start = 0, uint32_t loop_end = 0) override {
    _loop_start = loop_start;
    _loop_end = loop_end;

    _wavFile.open(file_path, std::ios::binary);

    if (!_wavFile.is_open()) {
      std::cerr << "Error: Cannot open input file: " << file_path << std::endl;
      return;
    }

    // Read WAV header
    WAVHeader header;
    _wavFile.read(reinterpret_cast<char *>(&header.riff), 4);
    header.chunkSize = readUint32LE(_wavFile);
    _wavFile.read(reinterpret_cast<char *>(&header.wave), 4);

    // Validate RIFF header
    if (std::strncmp(header.riff, "RIFF", 4) != 0 || std::strncmp(header.wave, "WAVE", 4) != 0) {
      std::cerr << "Error: Invalid WAV file format: " << file_path << std::endl;
      return;
    }

    // Find and read fmt chunk
    WAVFmtChunk fmtChunk;
    bool foundFmt = false;
    char chunkId[4];

    while (_wavFile.read(chunkId, 4)) {
      if (std::strncmp(chunkId, "fmt ", 4) == 0) {
        foundFmt = true;
        fmtChunk.chunkSize = readUint32LE(_wavFile);
        fmtChunk.audioFormat = readUint16LE(_wavFile);
        fmtChunk.numChannels = readUint16LE(_wavFile);
        fmtChunk.sampleRate = readUint32LE(_wavFile);
        fmtChunk.byteRate = readUint32LE(_wavFile);
        fmtChunk.blockAlign = readUint16LE(_wavFile);
        fmtChunk.bitsPerSample = readUint16LE(_wavFile);
        break;
      } else {
        // Skip this chunk
        uint32_t chunkSize = readUint32LE(_wavFile);
        _wavFile.seekg(chunkSize, std::ios::cur);
      }
    }

    if (!foundFmt) {
      std::cerr << "Error: fmt chunk not found in: " << file_path << std::endl;
      return;
    }

    // Validate format
    if (fmtChunk.audioFormat != 1) {
      std::cerr << "Error: Only PCM format is supported (found format " << fmtChunk.audioFormat << "): " << file_path
                << std::endl;
      return;
    }

    if (fmtChunk.sampleRate != 44100) {
      std::cerr << "Error: Sample rate must be 44100 Hz (found " << fmtChunk.sampleRate << " Hz): " << file_path
                << std::endl;
      return;
    }

    if (fmtChunk.bitsPerSample != 16) {
      std::cerr << "Error: Bit depth must be 16-bit (found " << fmtChunk.bitsPerSample << "-bit): " << file_path
                << std::endl;
      return;
    }

    if (fmtChunk.numChannels != 1) {
      std::cerr << "Error: Only mono channel supported (found " << fmtChunk.numChannels << " channels): " << file_path
                << std::endl;
      return;
    }

    // Find data chunk
    WAVDataChunk dataChunk;
    bool foundData = false;

    while (_wavFile.read(chunkId, 4)) {
      if (std::strncmp(chunkId, "data", 4) == 0) {
        foundData = true;
        dataChunk.chunkSize = readUint32LE(_wavFile);
        break;
      } else {
        // Skip this chunk
        uint32_t chunkSize = readUint32LE(_wavFile);
        _wavFile.seekg(chunkSize, std::ios::cur);
      }
    }

    if (!foundData) {
      std::cerr << "Error: data chunk not found in: " << file_path << std::endl;
      return;
    }

    _samplesStartPos = _wavFile.tellg();
    _samplesCount = dataChunk.chunkSize / (fmtChunk.bitsPerSample / 8);
  }

  void deinit() override {
    if (!_wavFile.is_open()) {
      _wavFile.close();
    }
  }

  size_t get_samples_count() override {
    return _samplesCount;
  }

  int16_t read_sample(uint32_t index) override {
    _wavFile.seekg(_samplesStartPos + index * sizeof(uint16_t));
    return readInt16LE(_wavFile);
  }

  int16_t read_next_sample(bool is_looping = false) override {
    if (is_looping) {
      if (_wavFile.tellg() == _samplesStartPos + _loop_end * sizeof(uint16_t)) {
        _wavFile.seekg(_samplesStartPos + _loop_start * sizeof(uint16_t));
      }
    } else if (is_end()) {
      reset();
    }

    return readInt16LE(_wavFile);
  }

  void reset() override {
    _wavFile.seekg(_samplesStartPos);
  }

  bool is_start() override {
    return _wavFile.tellg() == _samplesStartPos;
  }

  bool is_end() override {
    auto last_sample_index = _samplesStartPos + _samplesCount * sizeof(uint16_t);
    return _wavFile.tellg() == last_sample_index;
  }

private:
  uint16_t _buffer[512];
  std::ifstream _wavFile;
  size_t _samplesCount;
  int32_t _samplesStartPos;
  int32_t _loop_start;
  int32_t _loop_end;
};