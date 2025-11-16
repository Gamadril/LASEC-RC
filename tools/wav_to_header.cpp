#include <algorithm>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace fs = std::filesystem;

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

// Read little-endian 16-bit value
int16_t readInt16LE(std::ifstream &file) {
  uint8_t bytes[2];
  file.read(reinterpret_cast<char *>(bytes), 2);
  return static_cast<int16_t>(bytes[0] | (bytes[1] << 8));
}

// Read little-endian 32-bit value
uint32_t readUint32LE(std::ifstream &file) {
  uint8_t bytes[4];
  file.read(reinterpret_cast<char *>(bytes), 4);
  return bytes[0] | (bytes[1] << 8) | (bytes[2] << 16) | (bytes[3] << 24);
}

// Read little-endian 16-bit value
uint16_t readUint16LE(std::ifstream &file) {
  uint8_t bytes[2];
  file.read(reinterpret_cast<char *>(bytes), 2);
  return bytes[0] | (bytes[1] << 8);
}

// Convert WAV file to header file
bool convertWavToHeader(const std::string &inputPath, const std::string &outputPath) {
  std::ifstream wavFile(inputPath, std::ios::binary);
  if (!wavFile.is_open()) {
    std::cerr << "Error: Cannot open input file: " << inputPath << std::endl;
    return false;
  }

  // Read WAV header
  WAVHeader header;
  wavFile.read(reinterpret_cast<char *>(&header.riff), 4);
  header.chunkSize = readUint32LE(wavFile);
  wavFile.read(reinterpret_cast<char *>(&header.wave), 4);

  // Validate RIFF header
  if (std::strncmp(header.riff, "RIFF", 4) != 0 || std::strncmp(header.wave, "WAVE", 4) != 0) {
    std::cerr << "Error: Invalid WAV file format: " << inputPath << std::endl;
    return false;
  }

  // Find and read fmt chunk
  WAVFmtChunk fmtChunk;
  bool foundFmt = false;
  char chunkId[4];

  while (wavFile.read(chunkId, 4)) {
    if (std::strncmp(chunkId, "fmt ", 4) == 0) {
      foundFmt = true;
      fmtChunk.chunkSize = readUint32LE(wavFile);
      fmtChunk.audioFormat = readUint16LE(wavFile);
      fmtChunk.numChannels = readUint16LE(wavFile);
      fmtChunk.sampleRate = readUint32LE(wavFile);
      fmtChunk.byteRate = readUint32LE(wavFile);
      fmtChunk.blockAlign = readUint16LE(wavFile);
      fmtChunk.bitsPerSample = readUint16LE(wavFile);

      // Skip any extra bytes in fmt chunk (if chunkSize > 16)
      if (fmtChunk.chunkSize > 16) {
        wavFile.seekg(fmtChunk.chunkSize - 16, std::ios::cur);
      }
      break;
    } else {
      // Skip this chunk
      uint32_t chunkSize = readUint32LE(wavFile);
      wavFile.seekg(chunkSize, std::ios::cur);
    }
  }

  if (!foundFmt) {
    std::cerr << "Error: fmt chunk not found in: " << inputPath << std::endl;
    return false;
  }

  // Validate format
  if (fmtChunk.audioFormat != 1) {
    std::cerr << "Error: Only PCM format is supported (found format " << fmtChunk.audioFormat
              << "): " << inputPath << std::endl;
    return false;
  }

  if (fmtChunk.sampleRate != 44100) {
    std::cerr << "Error: Sample rate must be 44100 Hz (found " << fmtChunk.sampleRate
              << " Hz): " << inputPath << std::endl;
    return false;
  }

  if (fmtChunk.bitsPerSample != 16) {
    std::cerr << "Error: Bit depth must be 16-bit (found " << fmtChunk.bitsPerSample
              << "-bit): " << inputPath << std::endl;
    return false;
  }

  if (fmtChunk.numChannels != 1) {
    std::cerr << "Error: Only mono channel supported (found " << fmtChunk.numChannels
              << " channels): " << inputPath << std::endl;
    return false;
  }

  // Find data chunk
  WAVDataChunk dataChunk;
  bool foundData = false;

  while (wavFile.read(chunkId, 4)) {
    if (std::strncmp(chunkId, "data", 4) == 0) {
      foundData = true;
      dataChunk.chunkSize = readUint32LE(wavFile);
      break;
    } else {
      // Skip this chunk
      uint32_t chunkSize = readUint32LE(wavFile);
      wavFile.seekg(chunkSize, std::ios::cur);
    }
  }

  if (!foundData) {
    std::cerr << "Error: data chunk not found in: " << inputPath << std::endl;
    return false;
  }

  // Read audio data
  size_t numSamples = dataChunk.chunkSize / (fmtChunk.bitsPerSample / 8);
  if (fmtChunk.numChannels == 2) {
    // Stereo: take only left channel or mix down
    numSamples = numSamples / 2;
  }

  std::vector<int16_t> samples;
  samples.reserve(numSamples);

  for (size_t i = 0; i < numSamples; i++) {
    int16_t sample = readInt16LE(wavFile);

    if (fmtChunk.numChannels == 2) {
      // Stereo: read right channel and average
      int16_t rightSample = readInt16LE(wavFile);
      sample = (sample + rightSample) / 2;
    }

    samples.push_back(sample);
  }

  wavFile.close();

  // Generate output filename
  std::string basename = fs::path(inputPath).stem().string();
  std::string outputFile = (fs::path(outputPath) / (basename + ".h")).string();

  // Write header file
  std::ofstream headerFile(outputFile);
  if (!headerFile.is_open()) {
    std::cerr << "Error: Cannot create output file: " << outputFile << std::endl;
    return false;
  }

  // Convert basename to uppercase for header guard
  std::string guardName = basename;
  std::transform(guardName.begin(), guardName.end(), guardName.begin(), ::toupper);
  std::replace(guardName.begin(), guardName.end(), '-', '_');
  std::replace(guardName.begin(), guardName.end(), '.', '_');
  std::string varName = basename + "_samples";

  // Write header file
  headerFile << "#pragma once\n\n";
  headerFile << "const int16_t " << varName << "[] = {\n";

  // Write samples with formatting (16 per line, matching existing header style)
  for (size_t i = 0; i < samples.size(); i++) {
    if (i % 16 == 0) {
      headerFile << "    ";
    }
    headerFile << std::setw(6) << samples[i];
    if (i < samples.size() - 1) {
      headerFile << ",";
    }
    if (i % 16 == 15 || i == samples.size() - 1) {
      headerFile << "\n";
    } else {
      headerFile << " ";
    }
  }

  headerFile << "};\n";

  headerFile << "const size_t " << varName << "_length = sizeof(" << varName << ") / sizeof("
             << varName << "[0]);\n";

  headerFile.close();

  std::cout << "Converted: " << inputPath << " -> " << outputFile << " (" << samples.size()
            << " samples)" << std::endl;
  return true;
}

// Process single file or directory
bool processInput(const std::string &inputPath, const std::string &outputPath) {
  if (!fs::exists(inputPath)) {
    std::cerr << "Error: Input path does not exist: " << inputPath << std::endl;
    return false;
  }

  // Create output directory if it doesn't exist
  if (!fs::exists(outputPath)) {
    if (!fs::create_directories(outputPath)) {
      std::cerr << "Error: Cannot create output directory: " << outputPath << std::endl;
      return false;
    }
  }

  std::vector<std::string> wavFiles;

  if (fs::is_regular_file(inputPath)) {
    // Single file
    fs::path inputFilePath(inputPath);
    std::string ext = inputFilePath.extension().string();
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
    if (ext == ".wav") {
      wavFiles.push_back(inputPath);
    } else {
      std::cerr << "Error: Input file is not a .wav file: " << inputPath << std::endl;
      return false;
    }
  } else if (fs::is_directory(inputPath)) {
    // Directory: find all .wav files
    for (const auto &entry : fs::directory_iterator(inputPath)) {
      if (entry.is_regular_file()) {
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
        if (ext == ".wav") {
          wavFiles.push_back(entry.path().string());
        }
      }
    }
  } else {
    std::cerr << "Error: Input path is neither a file nor a directory: " << inputPath << std::endl;
    return false;
  }

  if (wavFiles.empty()) {
    std::cerr << "Error: No .wav files found in: " << inputPath << std::endl;
    return false;
  }

  bool allSuccess = true;
  for (const auto &wavFile : wavFiles) {
    if (!convertWavToHeader(wavFile, outputPath)) {
      allSuccess = false;
    }
  }

  return allSuccess;
}

int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cerr << "Usage: " << argv[0] << " <input_file_or_folder> <output_folder>" << std::endl;
    std::cerr << "  Converts .wav files to C header files with signed short arrays." << std::endl;
    std::cerr << "  Input files must be 44100 Hz, 16-bit PCM, mono." << std::endl;
    return 1;
  }

  std::string inputPath = argv[1];
  std::string outputPath = argv[2];

  if (!processInput(inputPath, outputPath)) {
    return 1;
  }

  return 0;
}
