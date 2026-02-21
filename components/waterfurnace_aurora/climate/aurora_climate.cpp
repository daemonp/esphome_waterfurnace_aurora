#include "aurora_climate.h"
#include "aurora_climate_utils.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // for fahrenheit_to_celsius / celsius_to_fahrenheit

#include <cmath>

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.climate";

void AuroraClimate::setup() {
  // Register with parent to receive data update notifications.
  // Replaces the 5-second polling loop — update_state_() is called
  // immediately when fresh register data arrives from the heat pump.
  if (this->parent_ != nullptr) {
    this->parent_->register_listener([this]() { this->update_state_(); });
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
  
  // Temperature ranges from thermostat.rb (converted to Celsius for ESPHome)
  // Heating min: 40°F = 4.4°C, Cooling max: 99°F = 37.2°C
  traits.set_visual_min_temperature(fahrenheit_to_celsius(40));
  traits.set_visual_max_temperature(fahrenheit_to_celsius(99));
  traits.set_visual_temperature_step(0.5);
  
  return traits;
}

void AuroraClimate::control(const climate::ClimateCall &call) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control");
    return;
  }

  ESP_LOGD(TAG, "control() called: has_mode=%d has_temp_low=%d has_temp_high=%d has_fan=%d has_preset=%d",
           call.get_mode().has_value(),
           call.get_target_temperature_low().has_value(),
           call.get_target_temperature_high().has_value(),
           call.get_fan_mode().has_value(),
           call.get_preset().has_value());

  // Handle mode change
  if (call.get_mode().has_value()) {
    HeatingMode aurora_mode;
    if (!esphome_to_aurora_mode(*call.get_mode(), aurora_mode)) {
      ESP_LOGW(TAG, "Unsupported mode");
      return;
    }
    if (this->parent_->set_hvac_mode(aurora_mode)) {
      this->mode = *call.get_mode();
    }
  }

  // Handle target temperature changes (dual setpoint)
  // HA sends Celsius; hardware expects Fahrenheit
  if (call.get_target_temperature_low().has_value()) {
    float temp_c = *call.get_target_temperature_low();
    float temp_f = celsius_to_fahrenheit(temp_c);
    ESP_LOGD(TAG, "Heating setpoint: HA sent %.2f°C = %.1f°F, current cached=%.1f°F",
             temp_c, temp_f, this->parent_->get_heating_setpoint());
    if (this->parent_->set_heating_setpoint(temp_f)) {
      this->target_temperature_low = temp_c;
      ESP_LOGD(TAG, "Heating setpoint accepted: hub cached now=%.1f°F", this->parent_->get_heating_setpoint());
    }
  }
  
  if (call.get_target_temperature_high().has_value()) {
    float temp_c = *call.get_target_temperature_high();
    float temp_f = celsius_to_fahrenheit(temp_c);
    ESP_LOGD(TAG, "Cooling setpoint: HA sent %.2f°C = %.1f°F, current cached=%.1f°F",
             temp_c, temp_f, this->parent_->get_cooling_setpoint());
    if (this->parent_->set_cooling_setpoint(temp_f)) {
      this->target_temperature_high = temp_c;
      ESP_LOGD(TAG, "Cooling setpoint accepted: hub cached now=%.1f°F", this->parent_->get_cooling_setpoint());
    }
  }

  // Handle fan mode
  if (call.get_fan_mode().has_value()) {
    FanMode aurora_fan = esphome_to_aurora_fan(*call.get_fan_mode());
    if (this->parent_->set_fan_mode(aurora_fan)) {
      this->fan_mode = *call.get_fan_mode();
    }
  }

  // Handle preset (emergency heat)
  if (call.get_preset().has_value()) {
    climate::ClimatePreset preset = *call.get_preset();
    
    if (preset == climate::CLIMATE_PRESET_BOOST) {
      // BOOST preset = Emergency Heat mode
      if (this->parent_->set_hvac_mode(HeatingMode::EHEAT)) {
        this->preset = preset;
        this->mode = climate::CLIMATE_MODE_HEAT;
      }
    } else if (preset == climate::CLIMATE_PRESET_NONE) {
      // Clear preset - if we were in E-Heat, switch back to regular heat
      if (this->parent_->get_hvac_mode() == HeatingMode::EHEAT) {
        this->parent_->set_hvac_mode(HeatingMode::HEAT);
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

  // Update current temperature (hardware reports °F, climate entity uses °C)
  float ambient = this->parent_->get_ambient_temperature();
  if (!std::isnan(ambient)) {
    this->current_temperature = fahrenheit_to_celsius(ambient);
  }

  // Update setpoints (hardware reports °F, climate entity uses °C)
  float heating_sp = this->parent_->get_heating_setpoint();
  float cooling_sp = this->parent_->get_cooling_setpoint();
  
  if (!std::isnan(heating_sp)) {
    float new_low = fahrenheit_to_celsius(heating_sp);
    if (this->target_temperature_low != new_low) {
      ESP_LOGD(TAG, "update_state_: heating SP changing %.2f°C -> %.2f°C (hub=%.1f°F)",
               this->target_temperature_low, new_low, heating_sp);
    }
    this->target_temperature_low = new_low;
  }
  if (!std::isnan(cooling_sp)) {
    float new_high = fahrenheit_to_celsius(cooling_sp);
    if (this->target_temperature_high != new_high) {
      ESP_LOGD(TAG, "update_state_: cooling SP changing %.2f°C -> %.2f°C (hub=%.1f°F)",
               this->target_temperature_high, new_high, cooling_sp);
    }
    this->target_temperature_high = new_high;
  }

  // Update mode and preset
  climate::ClimateMode new_mode;
  climate::ClimatePreset new_preset;
  if (aurora_to_esphome_mode(this->parent_->get_hvac_mode(), new_mode, new_preset)) {
    this->mode = new_mode;
    this->preset = new_preset;
  }

  // Update action based on system outputs
  uint16_t outputs = this->parent_->get_system_outputs();
  bool compressor = (outputs & (OUTPUT_CC | OUTPUT_CC2)) != 0;
  bool cooling = (outputs & OUTPUT_RV) != 0;
  bool aux_heat = (outputs & (OUTPUT_EH1 | OUTPUT_EH2)) != 0;
  bool blower = (outputs & OUTPUT_BLOWER) != 0;
  
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
  this->fan_mode = aurora_to_esphome_fan(this->parent_->get_fan_mode());

  this->publish_state();
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
