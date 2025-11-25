#include "aurora_button.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.button";

void AuroraClearFaultButton::setup() {
  // Nothing specific to set up
}

void AuroraClearFaultButton::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora Clear Fault History Button:");
}

void AuroraClearFaultButton::press_action() {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot clear fault history");
    return;
  }
  
  if (this->parent_->clear_fault_history()) {
    ESP_LOGI(TAG, "Fault history cleared successfully");
  } else {
    ESP_LOGW(TAG, "Failed to clear fault history");
  }
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
