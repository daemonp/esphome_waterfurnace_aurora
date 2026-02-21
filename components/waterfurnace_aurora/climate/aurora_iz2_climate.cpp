#include "aurora_iz2_climate.h"
#include "aurora_climate_utils.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.iz2_climate";

void AuroraIZ2Climate::setup() {
  // Register with parent to receive data update notifications.
  if (this->parent_ != nullptr) {
    this->parent_->register_listener([this]() { this->update_state_(); });
  }
}

void AuroraIZ2Climate::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora IZ2 Zone %d Climate:", this->zone_number_);
  ESP_LOGCONFIG(TAG, "  Parent: %s", this->parent_ != nullptr ? "configured" : "NOT SET");
}

climate::ClimateTraits AuroraIZ2Climate::traits() {
  auto traits = climate::ClimateTraits();
  
  // Feature flags
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  traits.add_feature_flags(climate::CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE);
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
  
  // Supported modes from registers.rb HEATING_MODE
  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);  // Auto
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
  
  // Supported fan modes from registers.rb FAN_MODE
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_ON);  // Continuous
  
  // Presets - use BOOST for emergency heat
  traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);
  traits.add_supported_preset(climate::CLIMATE_PRESET_BOOST);  // Emergency Heat
  
  // Temperature ranges from thermostat.rb / iz2_zone.rb
  // Heating: 40-90, Cooling: 54-99
  traits.set_visual_min_temperature(40);
  traits.set_visual_max_temperature(99);
  traits.set_visual_temperature_step(0.5);
  
  return traits;
}

void AuroraIZ2Climate::control(const climate::ClimateCall &call) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control zone %d", this->zone_number_);
    return;
  }

  if (!this->parent_->has_iz2()) {
    ESP_LOGW(TAG, "IZ2 not detected, cannot control zone %d", this->zone_number_);
    return;
  }

  if (this->zone_number_ > this->parent_->get_num_iz2_zones()) {
    ESP_LOGW(TAG, "Zone %d not available (only %d zones)", this->zone_number_, this->parent_->get_num_iz2_zones());
    return;
  }

  // Handle mode change
  if (call.get_mode().has_value()) {
    HeatingMode aurora_mode;
    if (!esphome_to_aurora_mode(*call.get_mode(), aurora_mode)) {
      ESP_LOGW(TAG, "Unsupported mode for zone %d", this->zone_number_);
      return;
    }
    if (this->parent_->set_zone_hvac_mode(this->zone_number_, aurora_mode)) {
      this->mode = *call.get_mode();
    }
  }

  // Handle target temperature changes (dual setpoint)
  if (call.get_target_temperature_low().has_value()) {
    float temp = *call.get_target_temperature_low();
    if (this->parent_->set_zone_heating_setpoint(this->zone_number_, temp)) {
      this->target_temperature_low = temp;
    }
  }
  
  if (call.get_target_temperature_high().has_value()) {
    float temp = *call.get_target_temperature_high();
    if (this->parent_->set_zone_cooling_setpoint(this->zone_number_, temp)) {
      this->target_temperature_high = temp;
    }
  }

  // Handle fan mode
  if (call.get_fan_mode().has_value()) {
    FanMode aurora_fan = esphome_to_aurora_fan(*call.get_fan_mode());
    if (this->parent_->set_zone_fan_mode(this->zone_number_, aurora_fan)) {
      this->fan_mode = *call.get_fan_mode();
    }
  }

  // Handle preset (emergency heat)
  if (call.get_preset().has_value()) {
    climate::ClimatePreset preset = *call.get_preset();
    
    if (preset == climate::CLIMATE_PRESET_BOOST) {
      // BOOST preset = Emergency Heat mode
      if (this->parent_->set_zone_hvac_mode(this->zone_number_, HeatingMode::EHEAT)) {
        this->preset = preset;
        this->mode = climate::CLIMATE_MODE_HEAT;
      }
    } else if (preset == climate::CLIMATE_PRESET_NONE) {
      // Clear preset - switch back to regular heat if we were in E-Heat
      const IZ2ZoneData& zone = this->parent_->get_zone_data(this->zone_number_);
      if (zone.target_mode == HeatingMode::EHEAT) {
        this->parent_->set_zone_hvac_mode(this->zone_number_, HeatingMode::HEAT);
      }
      this->preset = preset;
    }
  }

  this->publish_state();
}

void AuroraIZ2Climate::update_state_() {
  if (this->parent_ == nullptr) {
    return;
  }

  if (!this->parent_->has_iz2()) {
    return;
  }

  if (this->zone_number_ > this->parent_->get_num_iz2_zones()) {
    return;
  }

  const IZ2ZoneData& zone = this->parent_->get_zone_data(this->zone_number_);

  // Update current temperature
  if (!std::isnan(zone.ambient_temperature)) {
    this->current_temperature = zone.ambient_temperature;
  }

  // Update setpoints
  if (!std::isnan(zone.heating_setpoint)) {
    this->target_temperature_low = zone.heating_setpoint;
  }
  if (!std::isnan(zone.cooling_setpoint)) {
    this->target_temperature_high = zone.cooling_setpoint;
  }

  // Update mode and preset
  climate::ClimateMode new_mode;
  climate::ClimatePreset new_preset;
  if (aurora_to_esphome_mode(zone.target_mode, new_mode, new_preset)) {
    this->mode = new_mode;
    this->preset = new_preset;
  }

   // Update action based on zone current call and damper state
    if (!zone.damper_open) {
      this->action = climate::CLIMATE_ACTION_IDLE;
    } else {
      switch (zone.current_call) {
        case ZoneCall::H1:
        case ZoneCall::H2:
        case ZoneCall::H3:
          this->action = climate::CLIMATE_ACTION_HEATING;
          break;
        case ZoneCall::C1:
        case ZoneCall::C2:
          this->action = climate::CLIMATE_ACTION_COOLING;
          break;
        default:
          // STANDBY, UNKNOWN1, UNKNOWN7 → idle
          this->action = climate::CLIMATE_ACTION_IDLE;
      }
    }

  // Update fan mode
  this->fan_mode = aurora_to_esphome_fan(zone.target_fan_mode);

  this->publish_state();
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
