// Minimal ESPHome Sensor mock for unit testing
#pragma once
#include <cmath>

namespace esphome {
namespace sensor {

class Sensor {
 public:
  void publish_state(float state) {
    this->state = state;
    this->raw_state = state;
    this->has_state_ = true;
    this->publish_count_++;
  }

  bool has_state() const { return this->has_state_; }
  
  float state{NAN};
  float raw_state{NAN};
  bool has_state_{false};
  int publish_count_{0};
};

}  // namespace sensor
}  // namespace esphome
