#include "aurora_dhw_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.dhw_switch";

void AuroraDHWSwitch::setup() {
  // Register with parent to receive data update notifications.
  if (this->parent_ != nullptr) {
    this->parent_->register_listener([this]() { this->update_state_(); });
  }
}

void AuroraDHWSwitch::update_state_() {
  if (this->parent_ == nullptr) return;
  bool enabled = this->parent_->is_dhw_enabled();
  if (!this->has_published_ || enabled != this->last_state_) {
    this->publish_state(enabled);
    this->last_state_ = enabled;
    this->has_published_ = true;
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
