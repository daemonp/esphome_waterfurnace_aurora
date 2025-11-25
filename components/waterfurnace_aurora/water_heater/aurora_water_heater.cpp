#include "aurora_water_heater.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.water_heater";

void AuroraWaterHeater::setup() {
  // Nothing specific to set up
}

void AuroraWaterHeater::loop() {
  // Update state periodically
  uint32_t now = millis();
  if (now - this->last_update_ >= 5000) {  // Every 5 seconds
    this->update_state_();
    this->last_update_ = now;
  }
}

void AuroraWaterHeater::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora Water Heater (DHW):");
}

water_heater::WaterHeaterTraits AuroraWaterHeater::traits() {
  auto traits = water_heater::WaterHeaterTraits();
  
  traits.set_supports_current_temperature(true);
  traits.set_supports_target_temperature(true);
  
  // Supported modes: Off and Heat (from dhw.rb)
  traits.set_supported_modes({
    water_heater::WATER_HEATER_MODE_OFF,
    water_heater::WATER_HEATER_MODE_HEAT,
  });
  
  // Temperature range from dhw.rb: 100-140°F
  traits.set_visual_min_temperature(100);
  traits.set_visual_max_temperature(140);
  traits.set_visual_temperature_step(1);
  
  return traits;
}

void AuroraWaterHeater::control(const water_heater::WaterHeaterCall &call) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control");
    return;
  }

  // Handle mode change
  if (call.get_mode().has_value()) {
    water_heater::WaterHeaterMode mode = *call.get_mode();
    bool enabled = (mode != water_heater::WATER_HEATER_MODE_OFF);
    
    if (this->parent_->set_dhw_enabled(enabled)) {
      this->mode = mode;
    }
  }

  // Handle target temperature change
  if (call.get_target_temperature().has_value()) {
    float temp = *call.get_target_temperature();
    if (this->parent_->set_dhw_setpoint(temp)) {
      this->target_temperature = temp;
    }
  }

  this->publish_state();
}

void AuroraWaterHeater::update_state_() {
  if (this->parent_ == nullptr) {
    return;
  }

  // Update current temperature
  float dhw_temp = this->parent_->get_dhw_temperature();
  if (!std::isnan(dhw_temp)) {
    this->current_temperature = dhw_temp;
  }

  // Update target temperature (setpoint)
  float dhw_setpoint = this->parent_->get_dhw_setpoint();
  if (!std::isnan(dhw_setpoint)) {
    this->target_temperature = dhw_setpoint;
  }

  // Update mode based on enabled state
  if (this->parent_->is_dhw_enabled()) {
    this->mode = water_heater::WATER_HEATER_MODE_HEAT;
  } else {
    this->mode = water_heater::WATER_HEATER_MODE_OFF;
  }

  // Update action - check if DHW is actively running
  uint16_t axb_outputs = this->parent_->get_system_outputs();  // Note: This needs AXB outputs
  // We'll check the DHW running state from the parent
  // For now, determine action based on mode and temperature
  if (this->mode == water_heater::WATER_HEATER_MODE_OFF) {
    this->action = water_heater::WATER_HEATER_ACTION_OFF;
  } else if (this->current_temperature < this->target_temperature - 2.0f) {
    // DHW is likely heating if temp is significantly below setpoint
    this->action = water_heater::WATER_HEATER_ACTION_HEATING;
  } else {
    this->action = water_heater::WATER_HEATER_ACTION_IDLE;
  }

  this->publish_state();
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
