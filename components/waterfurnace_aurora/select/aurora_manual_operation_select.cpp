#include "aurora_manual_operation_select.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.manual_operation";

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
