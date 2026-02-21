#include "aurora_dhw_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.dhw_switch";

void AuroraDHWSwitch::loop() {
  // Update state periodically
  uint32_t now = millis();
  if (now - this->last_update_ >= 5000) {  // Every 5 seconds
    if (this->parent_ != nullptr) {
      bool enabled = this->parent_->is_dhw_enabled();
      if (!this->has_published_ || enabled != this->last_state_) {
        this->publish_state(enabled);
        this->last_state_ = enabled;
        this->has_published_ = true;
      }
    }
    this->last_update_ = now;
  }
}

void AuroraDHWSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora DHW Switch:");
  ESP_LOGCONFIG(TAG, "  Parent: %s", this->parent_ != nullptr ? "configured" : "NOT SET");
}

void AuroraDHWSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control DHW");
    return;
  }
  
  if (this->parent_->set_dhw_enabled(state)) {
    this->publish_state(state);
  }
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
