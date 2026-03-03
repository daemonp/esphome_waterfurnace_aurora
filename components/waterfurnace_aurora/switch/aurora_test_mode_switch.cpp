#include "aurora_test_mode_switch.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.test_mode_switch";

void AuroraTestModeSwitch::setup() {
  // Test mode is write-only — start in OFF state.
  this->publish_state(false);
}

void AuroraTestModeSwitch::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora Test Mode Switch:");
  ESP_LOGCONFIG(TAG, "  Parent: %s", this->parent_ != nullptr ? "configured" : "NOT SET");
}

void AuroraTestModeSwitch::write_state(bool state) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control test mode");
    return;
  }

  if (this->parent_->set_test_mode(state)) {
    this->publish_state(state);
  }
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
