#include "aurora_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.climate";

void AuroraClimate::loop() {
  // Update state periodically
  uint32_t now = millis();
  if (now - this->last_update_ >= 5000) {  // Every 5 seconds
    this->update_state_();
    this->last_update_ = now;
  }
}

void AuroraClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora Climate:");
  ESP_LOGCONFIG(TAG, "  Parent: %s", this->parent_ != nullptr ? "configured" : "NOT SET");
}

climate::ClimateTraits AuroraClimate::traits() {
  auto traits = climate::ClimateTraits();
  
  // Feature flags - use add_feature_flags() instead of deprecated set_supports_*() methods
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
  
  // Presets - use BOOST for emergency heat (there's no CLIMATE_PRESET_EMERGENCY_HEAT)
  traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);
  traits.add_supported_preset(climate::CLIMATE_PRESET_BOOST);  // Emergency Heat
  
  // Temperature ranges from thermostat.rb
  traits.set_visual_min_temperature(40);  // Heating min
  traits.set_visual_max_temperature(99);  // Cooling max
  traits.set_visual_temperature_step(0.5);
  
  return traits;
}

void AuroraClimate::control(const climate::ClimateCall &call) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control");
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
        ESP_LOGW(TAG, "Unsupported mode");
        return;
    }
    
    if (this->parent_->set_hvac_mode(aurora_mode)) {
      this->mode = mode;
    }
  }

  // Handle target temperature changes (dual setpoint)
  if (call.get_target_temperature_low().has_value()) {
    float temp = *call.get_target_temperature_low();
    if (this->parent_->set_heating_setpoint(temp)) {
      this->target_temperature_low = temp;
    }
  }
  
  if (call.get_target_temperature_high().has_value()) {
    float temp = *call.get_target_temperature_high();
    if (this->parent_->set_cooling_setpoint(temp)) {
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
    
    if (this->parent_->set_fan_mode(aurora_fan)) {
      this->fan_mode = fan_mode;
    }
  }

  // Handle preset (emergency heat)
  if (call.get_preset().has_value()) {
    climate::ClimatePreset preset = *call.get_preset();
    
    if (preset == climate::CLIMATE_PRESET_BOOST) {
      // BOOST preset = Emergency Heat mode
      if (this->parent_->set_hvac_mode(HEATING_MODE_EHEAT)) {
        this->preset = preset;
        this->mode = climate::CLIMATE_MODE_HEAT;
      }
    } else if (preset == climate::CLIMATE_PRESET_NONE) {
      // Clear preset - if we were in E-Heat, switch back to regular heat
      if (this->parent_->get_hvac_mode() == HEATING_MODE_EHEAT) {
        this->parent_->set_hvac_mode(HEATING_MODE_HEAT);
      }
      this->preset = preset;
    }
  }

  this->publish_state();
}

void AuroraClimate::update_state_() {
  if (this->parent_ == nullptr) {
    return;
  }

  // Warn if IZ2 is detected - the main climate entity reads system-wide registers
  // which are not meaningful on IZ2 systems. Use zone climate entities instead.
  if (this->parent_->has_iz2()) {
    static bool warned = false;
    if (!warned) {
      ESP_LOGW(TAG, "IZ2 detected - the main climate entity reads system-wide registers");
      ESP_LOGW(TAG, "which may show incorrect values on IZ2 systems.");
      ESP_LOGW(TAG, "Use the zone climate entities (zone: 1, zone: 2, etc.) instead");
      ESP_LOGW(TAG, "and consider removing or commenting out this main climate entity.");
      warned = true;
    }
  }

  // Update current temperature
  float ambient = this->parent_->get_ambient_temperature();
  if (!std::isnan(ambient)) {
    this->current_temperature = ambient;
  }

  // Update setpoints
  float heating_sp = this->parent_->get_heating_setpoint();
  float cooling_sp = this->parent_->get_cooling_setpoint();
  
  if (!std::isnan(heating_sp)) {
    this->target_temperature_low = heating_sp;
  }
  if (!std::isnan(cooling_sp)) {
    this->target_temperature_high = cooling_sp;
  }

  // Update mode and preset
  HeatingMode aurora_mode = this->parent_->get_hvac_mode();
  switch (aurora_mode) {
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

  // Update action based on system outputs
  uint16_t outputs = this->parent_->get_system_outputs();
  bool compressor = outputs & (OUTPUT_CC | OUTPUT_CC2);
  bool cooling = outputs & OUTPUT_RV;
  bool aux_heat = outputs & (OUTPUT_EH1 | OUTPUT_EH2);
  bool blower = outputs & OUTPUT_BLOWER;
  
  if (this->parent_->is_locked_out()) {
    this->action = climate::CLIMATE_ACTION_OFF;
  } else if (compressor && cooling) {
    this->action = climate::CLIMATE_ACTION_COOLING;
  } else if (compressor || aux_heat) {
    this->action = climate::CLIMATE_ACTION_HEATING;
  } else if (blower) {
    this->action = climate::CLIMATE_ACTION_FAN;
  } else {
    this->action = climate::CLIMATE_ACTION_IDLE;
  }

  // Update fan mode
  FanMode aurora_fan = this->parent_->get_fan_mode();
  switch (aurora_fan) {
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
