#include "aurora_humidistat_select.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.humidistat_select";

void AuroraHumidistatSelect::setup() {
  // Register with parent to receive data update notifications.
  if (this->parent_ != nullptr) {
    this->parent_->register_listener([this]() { this->update_state_(); });
  }
}

void AuroraHumidistatSelect::dump_config() {
  const char *type_str = (this->type_ == AuroraHumidistatType::HUMIDIFIER) ? "Humidifier" : "Dehumidifier";
  ESP_LOGCONFIG(TAG, "Aurora %s Mode Select:", type_str);
  ESP_LOGCONFIG(TAG, "  Parent: %s", this->parent_ != nullptr ? "configured" : "NOT SET");
}

void AuroraHumidistatSelect::update_state_() {
  if (this->parent_ == nullptr) return;

  bool is_auto;
  if (this->type_ == AuroraHumidistatType::HUMIDIFIER) {
    is_auto = this->parent_->get_humidifier_auto();
  } else {
    is_auto = this->parent_->get_dehumidifier_auto();
  }

  if (!this->has_published_ || is_auto != this->last_auto_) {
    this->publish_state(is_auto ? "Auto" : "Manual");
    this->last_auto_ = is_auto;
    this->has_published_ = true;
  }
}

void AuroraHumidistatSelect::control(const std::string &value) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control humidistat mode");
    return;
  }

  bool auto_mode = (value == "Auto");

  bool success;
  if (this->type_ == AuroraHumidistatType::HUMIDIFIER) {
    success = this->parent_->set_humidifier_mode(auto_mode);
  } else {
    success = this->parent_->set_dehumidifier_mode(auto_mode);
  }

  if (success) {
    this->publish_state(value);
  }
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
