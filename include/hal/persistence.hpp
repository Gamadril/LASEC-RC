#pragma once

#include <string>

class Persistence {
public:
  virtual ~Persistence() {
  }

  virtual void init(const std::string &path) = 0;
  virtual void deinit() = 0;
  /** @return true if data was loaded */
  virtual bool load(std::string &data) = 0;
  /** @return true if data was saved */
  virtual bool save(const std::string &data) = 0;
};
