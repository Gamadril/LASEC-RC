#pragma once

// Minimal sound data implementation - no file loading
#define SOUND_SAMPLE_RATE 22050

struct SoundData {
    uint8_t* data;
    size_t size;
    bool loaded;
    bool streaming;
    size_t currentPos;
    
    SoundData() : data(nullptr), size(0), loaded(false), streaming(false), currentPos(0) {}
    
    ~SoundData() {
        if (data) {
            free(data);
            data = nullptr;
        }
    }

    // Minimal implementation - no actual file loading
    bool loadFromFile(const char* filename) {
        // For now, just mark as not loaded since we don't have file system support
        loaded = false;
        size = 0;
        data = nullptr;
        return false;
    }

    bool loadFromFileStreaming(const char* filename) {
        // Minimal implementation - no streaming support
        return false;
    }

    int8_t getSample(size_t index) {
        // Return silence - no actual sound data
        return 0;
    }
    
    size_t getSize() const {
        return size;
    }
    
    bool isLoaded() const {
        return loaded;
    }
};

// Minimal SoundLoader - no filesystem operations
class SoundLoader {
public:
    static bool init() {
        // No filesystem initialization needed
        return true;
    }
    
    static void listFiles() {
        // No files to list
    }
    
    static void printSPIFFSInfo() {
        // No filesystem info to print
    }
    
    static void printMemoryInfo() {
        // No memory info to print
    }
    
    static size_t getFileSize(const char* filename) {
        return 0;
    }
    
    static bool fileExists(const char* filename) {
        return false;
    }
    
    static void checkSpecificFiles() {
        // No file checks needed
    }
};
