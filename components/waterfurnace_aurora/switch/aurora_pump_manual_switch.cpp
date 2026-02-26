#include "aurora_pump_manual_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.pump_manual_switch";

void AuroraPumpManualSwitch::setup() {
  // Register with parent to receive data update notifications.
  if (this->parent_ != nullptr) {
    this->parent_->register_listener([this]() { this->update_state_(); });
  }
}

void AuroraPumpManualSwitch::update_state_() {
  if (this->parent_ == nullptr) return;
  bool manual = this->parent_->is_pump_manual_control();
  if (!this->has_published_ || manual != this->last_state_) {
    this->publish_state(manual);
    this->last_state_ = manual;
    this->has_published_ = true;
  }
}

void AuroraPumpManualSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora VS Pump Manual Control Switch:");
  ESP_LOGCONFIG(TAG, "  Parent: %s", this->parent_ != nullptr ? "configured" : "NOT SET");
}

void AuroraPumpManualSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control pump manual mode");
    return;
  }
  
  if (this->parent_->set_pump_manual_control(state)) {
    this->publish_state(state);
  }
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
