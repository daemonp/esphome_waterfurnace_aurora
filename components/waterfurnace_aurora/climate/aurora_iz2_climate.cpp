#include "aurora_iz2_climate.h"
#include "aurora_climate_utils.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // for fahrenheit_to_celsius / celsius_to_fahrenheit

#include <cmath>

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
  
  // Presets - use BOOST for emergency heat
  traits.add_supported_preset(climate::CLIMATE_PRESET_NONE);
  traits.add_supported_preset(climate::CLIMATE_PRESET_BOOST);  // Emergency Heat
  
  // Temperature ranges from thermostat.rb / iz2_zone.rb (converted to Celsius for ESPHome)
  // Heating min: 40°F = 4.4°C, Cooling max: 99°F = 37.2°C
  traits.set_visual_min_temperature(fahrenheit_to_celsius(40));
  traits.set_visual_max_temperature(fahrenheit_to_celsius(99));
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
    // DRY mode is display-only — Aurora enters dehumidify automatically based on humidistat settings.
    // Skip only the mode write; fall through to handle any other fields in this call
    // (e.g., setpoints or fan mode sent in the same ClimateCall).
    if (*call.get_mode() == climate::CLIMATE_MODE_DRY) {
      ESP_LOGW(TAG, "DRY mode is display-only; ignoring mode change (other fields in this call still processed)");
    } else {
      HeatingMode aurora_mode;
      if (!esphome_to_aurora_mode(*call.get_mode(), aurora_mode)) {
        ESP_LOGW(TAG, "Unsupported mode for zone %d", this->zone_number_);
        return;
      }
      if (this->parent_->set_zone_hvac_mode(this->zone_number_, aurora_mode)) {
        this->mode = *call.get_mode();
      }
    }
  }

  // Handle target temperature changes (dual setpoint)
  // HA sends Celsius; hardware expects Fahrenheit
  if (call.get_target_temperature_low().has_value()) {
    float temp_c = *call.get_target_temperature_low();
    float temp_f = celsius_to_fahrenheit(temp_c);
    if (this->parent_->set_zone_heating_setpoint(this->zone_number_, temp_f)) {
      this->target_temperature_low = temp_c;
    }
  }
  
  if (call.get_target_temperature_high().has_value()) {
    float temp_c = *call.get_target_temperature_high();
    float temp_f = celsius_to_fahrenheit(temp_c);
    if (this->parent_->set_zone_cooling_setpoint(this->zone_number_, temp_f)) {
      this->target_temperature_high = temp_c;
    }
  }

  // Handle fan mode (built-in: Auto/On, or custom: Intermittent)
  if (call.get_fan_mode().has_value()) {
    // Built-in fan mode selected (Auto or On/Continuous)
    FanMode aurora_fan = esphome_to_aurora_fan(*call.get_fan_mode());
    if (this->parent_->set_zone_fan_mode(this->zone_number_, aurora_fan)) {
      this->fan_mode = *call.get_fan_mode();
    }
  } else if (call.has_custom_fan_mode()) {
    // Custom fan mode selected (Intermittent)
    if (this->parent_->set_zone_fan_mode(this->zone_number_, FanMode::INTERMITTENT)) {
      this->set_custom_fan_mode_(CUSTOM_FAN_MODE_INTERMITTENT);
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

  // Update current temperature (hardware reports °F, climate entity uses °C)
  if (!std::isnan(zone.ambient_temperature)) {
    this->current_temperature = fahrenheit_to_celsius(zone.ambient_temperature);
  }

  // Update current humidity (system-wide from register 741 — IZ2 zones share one sensor)
  float rh = this->parent_->get_relative_humidity();
  if (!std::isnan(rh)) {
    this->current_humidity = rh;
  }

  // Update setpoints (hardware reports °F, climate entity uses °C)
  // Skip during cooldown to preserve optimistic values from control()
  if (!this->parent_->setpoint_cooldown_active()) {
    if (!std::isnan(zone.heating_setpoint)) {
      this->target_temperature_low = fahrenheit_to_celsius(zone.heating_setpoint);
    }
    if (!std::isnan(zone.cooling_setpoint)) {
      this->target_temperature_high = fahrenheit_to_celsius(zone.cooling_setpoint);
    }
  }

  // Update mode and preset (skip during mode cooldown)
  if (!this->parent_->mode_cooldown_active()) {
    climate::ClimateMode new_mode;
    climate::ClimatePreset new_preset;
    if (aurora_to_esphome_mode(zone.target_mode, new_mode, new_preset)) {
      this->mode = new_mode;
      this->preset = new_preset;
    }
  }

  // Update action based on zone current call, damper state, and active dehumidification
  if (this->parent_->is_active_dehumidify()) {
    // VS Drive active dehumidification is system-wide — show as DRYING for all zones
    this->action = climate::CLIMATE_ACTION_DRYING;
  } else if (!zone.damper_open) {
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

  // Update fan mode (skip during fan cooldown)
  // Built-in modes (Auto/On) set fan_mode; Intermittent uses custom_fan_mode.
  if (!this->parent_->fan_cooldown_active()) {
    auto builtin = aurora_to_esphome_fan(zone.target_fan_mode);
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
