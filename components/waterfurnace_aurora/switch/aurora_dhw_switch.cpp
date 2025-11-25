#include "aurora_dhw_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.dhw_switch";

void AuroraDHWSwitch::setup() {
  // Nothing specific to set up
}

void AuroraDHWSwitch::loop() {
  // Update state periodically
  uint32_t now = millis();
  if (now - this->last_update_ >= 5000) {  // Every 5 seconds
    if (this->parent_ != nullptr) {
      bool enabled = this->parent_->is_dhw_enabled();
      this->publish_state(enabled);
    }
    this->last_update_ = now;
  }
}

void AuroraDHWSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora DHW Switch:");
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
