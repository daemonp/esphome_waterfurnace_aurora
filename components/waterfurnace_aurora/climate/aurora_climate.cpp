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
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_HUMIDITY);
  traits.add_feature_flags(climate::CLIMATE_REQUIRES_TWO_POINT_TARGET_TEMPERATURE);
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
  
  // Supported modes from registers.rb HEATING_MODE
  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);  // Auto
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
  traits.add_supported_mode(climate::CLIMATE_MODE_DRY);  // Active dehumidification (VS Drive)
  
  // Supported fan modes from registers.rb FAN_MODE
  // Auto and On (Continuous) are built-in ESPHome fan modes.
  // Intermittent is a WaterFurnace-specific mode exposed as a custom fan mode string.
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_AUTO);
  traits.add_supported_fan_mode(climate::CLIMATE_FAN_ON);  // Continuous
  traits.set_supported_custom_fan_modes({CUSTOM_FAN_MODE_INTERMITTENT});
  
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

  // Handle mode change
  if (call.get_mode().has_value()) {
    // DRY mode is display-only — Aurora enters dehumidify automatically based on humidistat settings
    if (*call.get_mode() == climate::CLIMATE_MODE_DRY) {
      ESP_LOGW(TAG, "DRY mode is automatic — controlled by the humidistat dehumidifier settings");
      return;
    }
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
    float temp_f = celsius_to_fahrenheit(*call.get_target_temperature_low());
    if (this->parent_->set_heating_setpoint(temp_f)) {
      this->target_temperature_low = *call.get_target_temperature_low();
    }
  }
  
  if (call.get_target_temperature_high().has_value()) {
    float temp_f = celsius_to_fahrenheit(*call.get_target_temperature_high());
    if (this->parent_->set_cooling_setpoint(temp_f)) {
      this->target_temperature_high = *call.get_target_temperature_high();
    }
  }

  // Handle fan mode (built-in: Auto/On, or custom: Intermittent)
  if (call.get_fan_mode().has_value()) {
    // Built-in fan mode selected (Auto or On/Continuous)
    FanMode aurora_fan = esphome_to_aurora_fan(*call.get_fan_mode());
    if (this->parent_->set_fan_mode(aurora_fan)) {
      this->fan_mode = *call.get_fan_mode();
    }
  } else if (call.has_custom_fan_mode()) {
    // Custom fan mode selected (Intermittent)
    if (this->parent_->set_fan_mode(FanMode::INTERMITTENT)) {
      this->set_custom_fan_mode_(CUSTOM_FAN_MODE_INTERMITTENT);
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
  if (this->parent_->has_iz2() && !this->iz2_warned_) {
    ESP_LOGW(TAG, "IZ2 detected - the main climate entity reads system-wide registers");
    ESP_LOGW(TAG, "which may show incorrect values on IZ2 systems.");
    ESP_LOGW(TAG, "Use the zone climate entities (zone: 1, zone: 2, etc.) instead");
    ESP_LOGW(TAG, "and consider removing or commenting out this main climate entity.");
    this->iz2_warned_ = true;
  }

  // Update current temperature (hardware reports °F, climate entity uses °C)
  float ambient = this->parent_->get_ambient_temperature();
  if (!std::isnan(ambient)) {
    this->current_temperature = fahrenheit_to_celsius(ambient);
  }

  // Update current humidity (register 741 — raw %)
  float rh = this->parent_->get_relative_humidity();
  if (!std::isnan(rh)) {
    this->current_humidity = rh;
  }

  // Update setpoints (hardware reports °F, climate entity uses °C)
  // Skip during write cooldown — prevents stale device read-back from
  // overwriting the optimistic value set by control().
  if (!this->parent_->setpoint_cooldown_active()) {
    float heating_sp = this->parent_->get_heating_setpoint();
    if (!std::isnan(heating_sp)) {
      this->target_temperature_low = fahrenheit_to_celsius(heating_sp);
    }
    float cooling_sp = this->parent_->get_cooling_setpoint();
    if (!std::isnan(cooling_sp)) {
      this->target_temperature_high = fahrenheit_to_celsius(cooling_sp);
    }
  }

  // Update mode and preset (skip during mode cooldown)
  if (!this->parent_->mode_cooldown_active()) {
    climate::ClimateMode new_mode;
    climate::ClimatePreset new_preset;
    if (aurora_to_esphome_mode(this->parent_->get_hvac_mode(), new_mode, new_preset)) {
      this->mode = new_mode;
      this->preset = new_preset;
    }
  }

  // Update action based on system outputs
  uint16_t outputs = this->parent_->get_system_outputs();
  bool compressor = (outputs & (OUTPUT_CC | OUTPUT_CC2)) != 0;
  bool cooling = (outputs & OUTPUT_RV) != 0;
  bool aux_heat = (outputs & (OUTPUT_EH1 | OUTPUT_EH2)) != 0;
  bool blower = (outputs & OUTPUT_BLOWER) != 0;
  bool dehumidifying = this->parent_->is_active_dehumidify();
  
  if (this->parent_->is_locked_out()) {
    this->action = climate::CLIMATE_ACTION_OFF;
  } else if (dehumidifying) {
    // VS Drive active dehumidification — show as DRYING
    this->action = climate::CLIMATE_ACTION_DRYING;
  } else if (compressor && cooling) {
    this->action = climate::CLIMATE_ACTION_COOLING;
  } else if (compressor || aux_heat) {
    this->action = climate::CLIMATE_ACTION_HEATING;
  } else if (blower) {
    this->action = climate::CLIMATE_ACTION_FAN;
  } else {
    this->action = climate::CLIMATE_ACTION_IDLE;
  }

  // Update fan mode (skip during fan cooldown)
  // Built-in modes (Auto/On) set fan_mode; Intermittent uses custom_fan_mode.
  if (!this->parent_->fan_cooldown_active()) {
    FanMode aurora_fan = this->parent_->get_fan_mode();
    auto builtin = aurora_to_esphome_fan(aurora_fan);
    if (builtin.has_value()) {
      this->fan_mode = *builtin;
    } else {
      // Intermittent — set via protected custom fan mode API
      this->set_custom_fan_mode_(CUSTOM_FAN_MODE_INTERMITTENT);
    }
  }

  this->publish_state();
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
