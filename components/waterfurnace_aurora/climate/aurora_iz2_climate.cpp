#include "aurora_iz2_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.iz2_climate";

void AuroraIZ2Climate::setup() {
  // Nothing specific to set up
}

void AuroraIZ2Climate::loop() {
  // Update state periodically
  uint32_t now = millis();
  if (now - this->last_update_ >= 5000) {  // Every 5 seconds
    this->update_state_();
    this->last_update_ = now;
  }
}

void AuroraIZ2Climate::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora IZ2 Zone %d Climate:", this->zone_number_);
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
    climate::ClimateMode mode = *call.get_mode();
    HeatingMode aurora_mode;
    
    switch (mode) {
      case climate::CLIMATE_MODE_OFF:
        aurora_mode = HEATING_MODE_OFF;
        break;
      case climate::CLIMATE_MODE_HEAT_COOL:
        aurora_mode = HEATING_MODE_AUTO;
        break;
      case climate::CLIMATE_MODE_COOL:
        aurora_mode = HEATING_MODE_COOL;
        break;
      case climate::CLIMATE_MODE_HEAT:
        aurora_mode = HEATING_MODE_HEAT;
        break;
      default:
        ESP_LOGW(TAG, "Unsupported mode for zone %d", this->zone_number_);
        return;
    }
    
    if (this->parent_->set_zone_hvac_mode(this->zone_number_, aurora_mode)) {
      this->mode = mode;
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
    climate::ClimateFanMode fan_mode = *call.get_fan_mode();
    FanMode aurora_fan;
    
    switch (fan_mode) {
      case climate::CLIMATE_FAN_AUTO:
        aurora_fan = FAN_MODE_AUTO;
        break;
      case climate::CLIMATE_FAN_ON:
        aurora_fan = FAN_MODE_CONTINUOUS;
        break;
      default:
        aurora_fan = FAN_MODE_AUTO;
    }
    
    if (this->parent_->set_zone_fan_mode(this->zone_number_, aurora_fan)) {
      this->fan_mode = fan_mode;
    }
  }

  // Handle preset (emergency heat)
  if (call.get_preset().has_value()) {
    climate::ClimatePreset preset = *call.get_preset();
    
    if (preset == climate::CLIMATE_PRESET_BOOST) {
      // BOOST preset = Emergency Heat mode
      if (this->parent_->set_zone_hvac_mode(this->zone_number_, HEATING_MODE_EHEAT)) {
        this->preset = preset;
        this->mode = climate::CLIMATE_MODE_HEAT;
      }
    } else if (preset == climate::CLIMATE_PRESET_NONE) {
      // Clear preset - switch back to regular heat if we were in E-Heat
      const IZ2ZoneData& zone = this->parent_->get_zone_data(this->zone_number_);
      if (zone.target_mode == HEATING_MODE_EHEAT) {
        this->parent_->set_zone_hvac_mode(this->zone_number_, HEATING_MODE_HEAT);
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
  switch (zone.target_mode) {
    case HEATING_MODE_OFF:
      this->mode = climate::CLIMATE_MODE_OFF;
      this->preset = climate::CLIMATE_PRESET_NONE;
      break;
    case HEATING_MODE_AUTO:
      this->mode = climate::CLIMATE_MODE_HEAT_COOL;
      this->preset = climate::CLIMATE_PRESET_NONE;
      break;
    case HEATING_MODE_COOL:
      this->mode = climate::CLIMATE_MODE_COOL;
      this->preset = climate::CLIMATE_PRESET_NONE;
      break;
    case HEATING_MODE_HEAT:
      this->mode = climate::CLIMATE_MODE_HEAT;
      this->preset = climate::CLIMATE_PRESET_NONE;
      break;
    case HEATING_MODE_EHEAT:
      this->mode = climate::CLIMATE_MODE_HEAT;
      this->preset = climate::CLIMATE_PRESET_BOOST;  // E-Heat shown as BOOST preset
      break;
  }

  // Update action based on zone current call and damper state
  // Zone call indicates what the zone is requesting
  if (!zone.damper_open) {
    // Damper closed - idle
    this->action = climate::CLIMATE_ACTION_IDLE;
  } else {
    // Damper open - determine action from current call
    switch (zone.current_call) {
      case ZONE_CALL_STANDBY:
        this->action = climate::CLIMATE_ACTION_IDLE;
        break;
      case ZONE_CALL_H1:
      case ZONE_CALL_H2:
      case ZONE_CALL_H3:
        this->action = climate::CLIMATE_ACTION_HEATING;
        break;
      case ZONE_CALL_C1:
      case ZONE_CALL_C2:
        this->action = climate::CLIMATE_ACTION_COOLING;
        break;
      default:
        this->action = climate::CLIMATE_ACTION_IDLE;
    }
  }

  // Update fan mode
  switch (zone.target_fan_mode) {
    case FAN_MODE_AUTO:
      this->fan_mode = climate::CLIMATE_FAN_AUTO;
      break;
    case FAN_MODE_CONTINUOUS:
    case FAN_MODE_INTERMITTENT:
      this->fan_mode = climate::CLIMATE_FAN_ON;
      break;
  }

  this->publish_state();
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
