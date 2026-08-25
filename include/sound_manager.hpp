#pragma once

#include "hal/wav_reader.hpp"
#include "sound_types.hpp"
#include "utils.hpp"
#include <dirent.h>
#include <functional>
#include <string>
#include <unordered_map>

class SoundManager {
public:
  using WavReaderFactory = std::function<WavReader *()>;

  SoundManager(WavReaderFactory factory) : _factory(factory) {
  }

  void scan(const char *path) {
    DIR *dir = opendir(path);
    if (!dir) {
      ESP_LOGE(TAG, "Failed to open directory %s", path);
      return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
      if (entry->d_type == DT_REG) { // Regular file
        std::string filename = entry->d_name;
        std::string nameWithoutExt = filename;

        // Remove extension
        size_t lastdot = nameWithoutExt.find_last_of(".");
        if (lastdot != std::string::npos) {
          nameWithoutExt = nameWithoutExt.substr(0, lastdot);
        }

        std::string soundName = nameWithoutExt;
        uint32_t loop_start = 0;
        uint32_t loop_end = 0;

        // Check for loop definition pattern: Name-Start-End
        // Find first hyphen
        size_t firstHyphen = nameWithoutExt.find('-');
        if (firstHyphen != std::string::npos) {
          // We have at least one hyphen, check for the second one
          size_t secondHyphen = nameWithoutExt.find('-', firstHyphen + 1);
          if (secondHyphen != std::string::npos) {
            // Format: Name-Start-End
            soundName = nameWithoutExt.substr(0, firstHyphen);
            std::string startStr = nameWithoutExt.substr(firstHyphen + 1, secondHyphen - firstHyphen - 1);
            std::string endStr = nameWithoutExt.substr(secondHyphen + 1);

            loop_start = std::stoul(startStr);
            loop_end = std::stoul(endStr);
          }
        }

        SoundType type = getSoundTypeFromFilename(soundName);
        if (type != SoundType::UNKNOWN) {
          std::string fullpath = std::string(path) + "/" + filename;

          WavReader *reader = _factory();
          reader->init(fullpath, loop_start, loop_end);
          _sounds[type] = reader;

          if (loop_start > 0 || loop_end > 0) {
            ESP_LOGI(TAG, "Loaded sound: %s as type %d (Loop: %u-%u)", filename.c_str(), (int)type, loop_start,
                     loop_end);
          } else {
            ESP_LOGI(TAG, "Loaded sound: %s as type %d", filename.c_str(), (int)type);
          }
        }
      }
    }
    closedir(dir);
  }

  std::unordered_map<SoundType, WavReader *> &getSounds() {
    return _sounds;
  }

private:
  WavReaderFactory _factory;
  std::unordered_map<SoundType, WavReader *> _sounds;
  static inline const char *TAG = "SND_MGR";
};
