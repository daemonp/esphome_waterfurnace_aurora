// Minimal ESPHome BinarySensor mock for unit testing
#pragma once

namespace esphome {
namespace binary_sensor {

class BinarySensor {
 public:
  void publish_state(bool state) {
    this->state = state;
    this->has_state_ = true;
    this->publish_count_++;
  }

  bool has_state() const { return this->has_state_; }
  
  bool state{false};
  bool has_state_{false};
  int publish_count_{0};
};

}  // namespace binary_sensor
}  // namespace esphome
