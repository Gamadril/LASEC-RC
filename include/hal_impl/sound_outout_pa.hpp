#pragma once

#include <cstdint>
#include <portaudio.h>

#include "../hal/sound_output.hpp"

class SoundOutputPA : public SoundOutput {
public:
  void init(uint16_t audio_rate) override {
    // Initialize PortAudio
    PaError err = Pa_Initialize();
    if (err != paNoError) {
      std::cerr << "PortAudio initialization failed: " << Pa_GetErrorText(err) << std::endl;
      return;
    }

    int numDevices = Pa_GetDeviceCount();
    if (numDevices < 0) {
      std::cerr << "Error getting device count: " << Pa_GetErrorText(numDevices) << std::endl;
      Pa_Terminate();
      return;
    }

    std::cout << "\nAvailable Audio Devices:\n";
    const PaDeviceInfo *deviceInfo;
    const PaHostApiInfo *hostApiInfo;
    int deviceIndex = 0;

    for (int i = 0; i < numDevices; i++) {
      deviceInfo = Pa_GetDeviceInfo(i);
      hostApiInfo = Pa_GetHostApiInfo(deviceInfo->hostApi);
      std::cout << "Device ID " << i << ": " << hostApiInfo->name << " - " << deviceInfo->name;
      if (deviceInfo->maxOutputChannels > 0) {
        std::cout << " [Output Channels: " << deviceInfo->maxOutputChannels << "]";
      }
      std::cout << std::endl;

      if (strcmp(deviceInfo->name, "pulse") == 0) {
        deviceIndex = i;
      }
    }

    PaStreamParameters outputParameters;
    outputParameters.device = deviceIndex;
    outputParameters.channelCount = 2;
    outputParameters.sampleFormat = paInt16;
    outputParameters.suggestedLatency = Pa_GetDeviceInfo(deviceIndex)->defaultLowOutputLatency;
    outputParameters.hostApiSpecificStreamInfo = NULL;

    err = Pa_OpenStream(&_stream,
                        NULL,              // Input
                        &outputParameters, // Output
                        audio_rate,        // Sample Rate
                        256,               // Frames Per Buffer
                        paClipOff,
                        &_audio_callback, // Your callback function
                        this);

    // default not always working correctly, problems with docking stations
    // err = Pa_OpenDefaultStream(&_stream, NULL, 2, paInt16, audio_rate, 256, &_audio_callback,
    // this);
    if (err != paNoError) {
      std::cerr << "PortAudio stream open failed: " << Pa_GetErrorText(err) << std::endl;
      Pa_Terminate();
      return;
    }

    err = Pa_StartStream(_stream);
    if (err != paNoError) {
      std::cerr << "PortAudio stream start failed: " << Pa_GetErrorText(err) << std::endl;
      Pa_CloseStream(_stream);
      Pa_Terminate();
    }
  }

  void deinit() override {
    // Cleanup
    Pa_StopStream(_stream);
    Pa_CloseStream(_stream);
    Pa_Terminate();
  }

private:
  PaStream *_stream;

  // PortAudio callback
  static int _audio_callback(const void *inputBuffer, void *outputBuffer,
                             unsigned long framesPerBuffer,
                             const PaStreamCallbackTimeInfo *timeInfo,
                             PaStreamCallbackFlags statusFlags, void *userData) {
    int16_t *out = static_cast<int16_t *>(outputBuffer);

    SoundOutputPA *instance = static_cast<SoundOutputPA *>(userData);

    if (instance && instance->_get_sample_func) {
      AudioSample sample;

      for (unsigned long i = 0; i < framesPerBuffer; i++) {
        instance->_get_sample_func(&sample, instance->_callback_user_data);
        *out++ = sample.left;
        *out++ = sample.right;
      }
    }

    return paContinue;
  }
};