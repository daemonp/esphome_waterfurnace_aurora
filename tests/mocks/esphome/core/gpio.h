// Minimal ESPHome GPIO mock for unit testing
#pragma once
#include <cstdint>

namespace esphome {

class GPIOPin {
 public:
  virtual ~GPIOPin() = default;
  virtual void setup() {}
  virtual void digital_write(bool value) { state_ = value; }
  virtual bool digital_read() { return state_; }
  bool state_{false};
};

}  // namespace esphome
