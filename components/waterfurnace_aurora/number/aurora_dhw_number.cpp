#include "aurora_dhw_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.dhw_number";

void AuroraDHWNumber::setup() {
  // Nothing specific to set up
}

void AuroraDHWNumber::loop() {
  // Update state periodically
  uint32_t now = millis();
  if (now - this->last_update_ >= 5000) {  // Every 5 seconds
    if (this->parent_ != nullptr) {
      float setpoint = this->parent_->get_dhw_setpoint();
      if (!std::isnan(setpoint)) {
        this->publish_state(setpoint);
      }
    }
    this->last_update_ = now;
  }
}

void AuroraDHWNumber::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora DHW Setpoint Number:");
}

void AuroraDHWNumber::control(float value) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control DHW setpoint");
    return;
  }
  
  if (this->parent_->set_dhw_setpoint(value)) {
    this->publish_state(value);
  }
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
