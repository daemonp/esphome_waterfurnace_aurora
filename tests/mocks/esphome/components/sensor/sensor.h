// Minimal ESPHome Sensor mock for unit testing
#pragma once
#include <cmath>

namespace esphome {
namespace sensor {

class Sensor {
 public:
  void publish_state(float state) {
    this->state = state;
    this->has_state_ = true;
    this->publish_count_++;
  }
  
  float state{NAN};
  bool has_state_{false};
  int publish_count_{0};
};

}  // namespace sensor
}  // namespace esphome
