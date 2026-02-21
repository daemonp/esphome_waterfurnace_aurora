// Minimal ESPHome TextSensor mock for unit testing
#pragma once
#include <string>

namespace esphome {
namespace text_sensor {

class TextSensor {
 public:
  void publish_state(const std::string &state) {
    this->state = state;
    this->has_state_ = true;
    this->publish_count_++;
  }
  
  std::string state;
  bool has_state_{false};
  int publish_count_{0};
};

}  // namespace text_sensor
}  // namespace esphome
