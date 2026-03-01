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

// ============================================================================
// Manual Operation Select
// ============================================================================

void AuroraManualOperationSelect::setup() {
  // No listener needed — manual mode is write-only (no read-back register to track).
  // The initial state is "Off".
  this->publish_state("Off");
}

void AuroraManualOperationSelect::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora Manual Operation Select:");
  ESP_LOGCONFIG(TAG, "  Parent: %s", this->parent_ != nullptr ? "configured" : "NOT SET");
  ESP_LOGCONFIG(TAG, "  Default compressor speed: %d", this->compressor_speed_);
  if (this->blower_speed_ == 255) {
    ESP_LOGCONFIG(TAG, "  Default blower speed: auto (with compressor)");
  } else {
    ESP_LOGCONFIG(TAG, "  Default blower speed: %d", this->blower_speed_);
  }
}

void AuroraManualOperationSelect::control(const std::string &value) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control manual operation");
    return;
  }

  bool success;
  if (value == "Off") {
    success = this->parent_->set_manual_operation_off();
  } else if (value == "Heating") {
    success = this->parent_->set_manual_operation(1, this->compressor_speed_, this->blower_speed_, false);
  } else if (value == "Cooling") {
    success = this->parent_->set_manual_operation(2, this->compressor_speed_, this->blower_speed_, false);
  } else if (value == "Heating+Aux") {
    success = this->parent_->set_manual_operation(1, this->compressor_speed_, this->blower_speed_, true);
  } else {
    ESP_LOGW(TAG, "Unknown manual operation mode: %s", value.c_str());
    return;
  }

  if (success) {
    this->publish_state(value);
  }
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
