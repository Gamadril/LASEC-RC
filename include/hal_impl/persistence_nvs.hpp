#pragma once

#include "../hal/persistence.hpp"

#include "esp_log.h"
#include "nvs_flash.h"
#include <vector>

class PersistenceNVS : public Persistence {
public:
  void init(const std::string &path) override {
    _path = path;
  }

  void deinit() override {
  }

  bool load(std::string &data) override {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("LASEC", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
      ESP_LOGW(TAG, "NVS open failed: %s", esp_err_to_name(err));
      return false;
    }

    size_t size = 0;
    err = nvs_get_blob(nvs_handle, _path.c_str(), nullptr, &size);
    if (err != ESP_OK || size == 0) {
      nvs_close(nvs_handle);
      return false;
    }

    std::vector<char> buf(size);
    err = nvs_get_blob(nvs_handle, _path.c_str(), buf.data(), &size);
    nvs_close(nvs_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "NVS read failed: %s", esp_err_to_name(err));
      return false;
    }

    data.assign(buf.data(), size);
    return true;
  }

  bool save(const std::string &data) override {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("LASEC", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "NVS open failed: %s", esp_err_to_name(err));
      return false;
    }

    err = nvs_set_blob(nvs_handle, _path.c_str(), data.data(), data.size());
    if (err == ESP_OK) {
      err = nvs_commit(nvs_handle);
    }
    nvs_close(nvs_handle);

    if (err != ESP_OK) {
      ESP_LOGE(TAG, "NVS save failed: %s", esp_err_to_name(err));
      return false;
    }
    return true;
  }

private:
  static inline const char *TAG = "NVS";
  std::string _path;
};
