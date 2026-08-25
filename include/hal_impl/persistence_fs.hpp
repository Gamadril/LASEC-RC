#pragma once

#include "../hal/persistence.hpp"

#include <fstream>
#include <iostream>
#include <sstream>

class PersistenceFS : public Persistence {
public:
  void init(const std::string &path) override {
    _path = path;
  }

  void deinit() override {
  }

  bool load(std::string &data) override {
    std::ifstream config_file(_path, std::ios::in);
    if (!config_file.is_open()) {
      return false;
    }
    std::ostringstream ss;
    ss << config_file.rdbuf();
    data = ss.str();
    return true;
  }

  bool save(const std::string &data) override {
    std::ofstream config_file(_path, std::ios::out | std::ios::trunc);
    if (!config_file.is_open()) {
      std::cerr << "Failed opening file for write: " << _path << std::endl;
      return false;
    }
    config_file << data;
    return true;
  }

private:
  std::string _path;
};
