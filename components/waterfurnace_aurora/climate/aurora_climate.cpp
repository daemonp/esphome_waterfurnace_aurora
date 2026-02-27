#include "aurora_climate.h"
#include "aurora_climate_utils.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"  // for fahrenheit_to_celsius / celsius_to_fahrenheit

#include <cmath>

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.climate";

void AuroraClimate::setup() {
  // Defer listener registration until hardware detection completes,
  // because is_iz2_mode_() depends on the result of has_iz2() detection.
  if (this->parent_->is_setup_complete()) {
    this->register_listeners_();
  } else {
    this->parent_->register_setup_callback(
        [this]() { this->register_listeners_(); });
  }
}

void AuroraClimate::register_listeners_() {
  ESP_LOGD(TAG, "Zone %d: registering as %s mode", this->zone_,
           this->is_iz2_mode_() ? "IZ2" : "thermostat");
  this->parent_->register_listener([this]() { this->update_state_(); });
}

void AuroraClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora Climate (zone %d):", this->zone_);
  ESP_LOGCONFIG(TAG, "  Parent: %s", this->parent_ != nullptr ? "configured" : "NOT SET");
}

climate::ClimateTraits AuroraClimate::traits() {
  auto traits = climate::ClimateTraits();
  
  // Feature flags - use add_feature_flags() instead of deprecated set_supports_*() methods
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_HUMIDITY);
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_TARGET_HUMIDITY);
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_TWO_POINT_TARGET_TEMPERATURE);
  traits.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
  
  // Supported modes from registers.rb HEATING_MODE
  // Note: DRY is not a selectable Aurora mode — active dehumidification is automatic
  // based on humidistat settings. CLIMATE_ACTION_DRYING shows when it's happening.
  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT_COOL);  // Auto
  traits.add_supported_mode(climate::CLIMATE_MODE_COOL);
  traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
  
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
  
  // Humidity target range — union of humidification (15-50%) and dehumidification (35-65%).
  // HA caches visual min/max from ListEntitiesClimateResponse (sent once at connection),
  // so we advertise the full range. control() clamps to the mode-appropriate sub-range.
  traits.set_visual_min_humidity(15);
  traits.set_visual_max_humidity(65);
  
  return traits;
}

void AuroraClimate::control(const climate::ClimateCall &call) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control zone %d", this->zone_);
    return;
  }

  if (this->is_iz2_mode_()) {
    if (!this->parent_->has_iz2()) {
      ESP_LOGW(TAG, "IZ2 not detected, cannot control zone %d", this->zone_);
      return;
    }
    if (this->zone_ > this->parent_->get_num_iz2_zones()) {
      ESP_LOGW(TAG, "Zone %d not available (only %d zones)", this->zone_,
               this->parent_->get_num_iz2_zones());
      return;
    }
  }

  // Handle mode change
  if (call.get_mode().has_value()) {
    HeatingMode aurora_mode;
    if (!esphome_to_aurora_mode(*call.get_mode(), aurora_mode)) {
      ESP_LOGW(TAG, "Unsupported mode for zone %d", this->zone_);
      return;
    }
    bool ok = this->is_iz2_mode_()
                  ? this->parent_->set_zone_hvac_mode(this->zone_, aurora_mode)
                  : this->parent_->set_hvac_mode(aurora_mode);
    if (ok) {
      this->mode = *call.get_mode();
    }
  }

  // Handle target temperature changes (dual setpoint)
  // HA sends Celsius; hardware expects Fahrenheit
  if (call.get_target_temperature_low().has_value()) {
    float temp_c = *call.get_target_temperature_low();
    float temp_f = celsius_to_fahrenheit(temp_c);
    bool ok = this->is_iz2_mode_()
                  ? this->parent_->set_zone_heating_setpoint(this->zone_, temp_f)
                  : this->parent_->set_heating_setpoint(temp_f);
    if (ok) {
      this->target_temperature_low = temp_c;
    }
  }
  
  if (call.get_target_temperature_high().has_value()) {
    float temp_c = *call.get_target_temperature_high();
    float temp_f = celsius_to_fahrenheit(temp_c);
    bool ok = this->is_iz2_mode_()
                  ? this->parent_->set_zone_cooling_setpoint(this->zone_, temp_f)
                  : this->parent_->set_cooling_setpoint(temp_f);
    if (ok) {
      this->target_temperature_high = temp_c;
    }
  }

  // Handle target humidity (mode-aware: humidification in heat, dehumidification in cool)
  // Humidistat is system-wide (not per-zone), but routing is per-zone based on zone mode.
  if (call.get_target_humidity().has_value()) {
    if (!this->parent_->is_setup_complete()) {
      ESP_LOGW(TAG, "Skipping humidity target write — setup not complete");
    } else {
      char prefix[32];
      snprintf(prefix, sizeof(prefix), "Zone %d: ", this->zone_);
      float out_target;
      if (route_humidity_write(this->parent_, this->mode, *call.get_target_humidity(),
                               out_target, prefix)) {
        this->target_humidity = out_target;
      }
    }
  }

  // Handle fan mode (built-in: Auto/On, or custom: Intermittent)
  if (call.get_fan_mode().has_value()) {
    // Built-in fan mode selected (Auto or On/Continuous)
    FanMode aurora_fan = esphome_to_aurora_fan(*call.get_fan_mode());
    bool ok = this->is_iz2_mode_()
                  ? this->parent_->set_zone_fan_mode(this->zone_, aurora_fan)
                  : this->parent_->set_fan_mode(aurora_fan);
    if (ok) {
      this->fan_mode = *call.get_fan_mode();
    }
  } else if (call.has_custom_fan_mode()) {
    // Custom fan mode selected (Intermittent)
    bool ok = this->is_iz2_mode_()
                  ? this->parent_->set_zone_fan_mode(this->zone_, FanMode::INTERMITTENT)
                  : this->parent_->set_fan_mode(FanMode::INTERMITTENT);
    if (ok) {
      this->set_custom_fan_mode_(CUSTOM_FAN_MODE_INTERMITTENT);
    }
  }

  // Handle preset (emergency heat)
  if (call.get_preset().has_value()) {
    climate::ClimatePreset preset = *call.get_preset();
    
    if (preset == climate::CLIMATE_PRESET_BOOST) {
      // BOOST preset = Emergency Heat mode
      bool ok = this->is_iz2_mode_()
                    ? this->parent_->set_zone_hvac_mode(this->zone_, HeatingMode::EHEAT)
                    : this->parent_->set_hvac_mode(HeatingMode::EHEAT);
      if (ok) {
        this->preset = preset;
        this->mode = climate::CLIMATE_MODE_HEAT;
      }
    } else if (preset == climate::CLIMATE_PRESET_NONE) {
      // Clear preset - if we were in E-Heat, switch back to regular heat
      if (this->is_iz2_mode_()) {
        const IZ2ZoneData& zone = this->parent_->get_zone_data(this->zone_);
        if (zone.target_mode == HeatingMode::EHEAT) {
          this->parent_->set_zone_hvac_mode(this->zone_, HeatingMode::HEAT);
        }
      } else {
        if (this->parent_->get_hvac_mode() == HeatingMode::EHEAT) {
          this->parent_->set_hvac_mode(HeatingMode::HEAT);
        }
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

  if (this->is_iz2_mode_()) {
    // === IZ2 path: read from per-zone data ===
    if (!this->parent_->has_iz2()) {
      return;
    }
    if (this->zone_ > this->parent_->get_num_iz2_zones()) {
      return;
    }

    const IZ2ZoneData& zone = this->parent_->get_zone_data(this->zone_);

    // Update current temperature (hardware reports °F, climate entity uses °C)
    if (!std::isnan(zone.ambient_temperature)) {
      this->current_temperature = fahrenheit_to_celsius(zone.ambient_temperature);
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

    // Update action based on zone damper and call state
    // XXX: Dehumidifier detection needs testing during summer cooling season.
    // See waterfurnace_aurora.cpp for full explanation of the reversing valve gate.
    bool cooling = (this->parent_->get_system_outputs() & OUTPUT_RV) != 0;
    bool dehumidifying = (this->parent_->is_active_dehumidify() && cooling)
                         || ((this->parent_->get_axb_outputs() & AXB_OUTPUT_DEHUMIDIFIER) != 0);
    if (!zone.damper_open) {
      // Zone damper closed — show DRYING only for standalone dehumidifier (AXB relay)
      this->action = dehumidifying ? climate::CLIMATE_ACTION_DRYING : climate::CLIMATE_ACTION_IDLE;
    } else {
      switch (zone.current_call) {
        case ZoneCall::H1:
        case ZoneCall::H2:
        case ZoneCall::H3:
          this->action = climate::CLIMATE_ACTION_HEATING;
          break;
        case ZoneCall::C1:
        case ZoneCall::C2:
          // During cooling, show DRYING if actively dehumidifying
          this->action = dehumidifying ? climate::CLIMATE_ACTION_DRYING : climate::CLIMATE_ACTION_COOLING;
          break;
        default:
          // STANDBY, UNKNOWN1, UNKNOWN7
          this->action = dehumidifying ? climate::CLIMATE_ACTION_DRYING : climate::CLIMATE_ACTION_IDLE;
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
  } else {
    // === Thermostat path: read from system-wide registers ===

    // Update current temperature (hardware reports °F, climate entity uses °C)
    float ambient = this->parent_->get_ambient_temperature();
    if (!std::isnan(ambient)) {
      this->current_temperature = fahrenheit_to_celsius(ambient);
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
    // XXX: Dehumidifier detection needs testing during summer cooling season.
    // See waterfurnace_aurora.cpp for full explanation of the reversing valve gate.
    bool dehumidifying = (this->parent_->is_active_dehumidify() && cooling)
                         || ((this->parent_->get_axb_outputs() & AXB_OUTPUT_DEHUMIDIFIER) != 0);
    
    if (this->parent_->is_locked_out()) {
      this->action = climate::CLIMATE_ACTION_OFF;
    } else if (compressor && cooling) {
      // During cooling, show DRYING if actively dehumidifying
      this->action = dehumidifying ? climate::CLIMATE_ACTION_DRYING : climate::CLIMATE_ACTION_COOLING;
    } else if (compressor || aux_heat) {
      this->action = climate::CLIMATE_ACTION_HEATING;
    } else if (dehumidifying) {
      // Standalone dehumidifier running (AXB relay) without compressor
      this->action = climate::CLIMATE_ACTION_DRYING;
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
  }

  // === Shared by both paths ===

  // Update current humidity (system-wide from register 741 — IZ2 zones share one sensor)
  float rh = this->parent_->get_relative_humidity();
  if (!std::isnan(rh)) {
    this->current_humidity = rh;
  }

  // Update target humidity (mode-aware: humidification in heat, dehumidification in cool)
  // Humidistat is system-wide — all IZ2 zones share the same targets, but each zone's
  // slider reflects the target relevant to that zone's operating mode.
  if (!this->parent_->humidity_target_cooldown_active()) {
    float humidity_target = read_mode_humidity_target(this->parent_, this->mode);
    if (!std::isnan(humidity_target)) {
      this->target_humidity = humidity_target;
    }
  }

  this->publish_state_if_changed_();
}

void AuroraClimate::publish_state_if_changed_() {
  // NaN-aware float comparison: NaN==NaN → unchanged, NaN↔real → changed.
  auto temp_changed = [](float a, float b) -> bool {
    if (std::isnan(a) && std::isnan(b))
      return false;
    if (std::isnan(a) || std::isnan(b))
      return true;
    return a != b;
  };

  if (this->has_published_) {
    bool changed = false;
    changed = changed || temp_changed(this->current_temperature, this->last_current_temp_);
    changed = changed || temp_changed(this->current_humidity, this->last_current_humidity_);
    changed = changed || temp_changed(this->target_temperature_low, this->last_target_low_);
    changed = changed || temp_changed(this->target_temperature_high, this->last_target_high_);
    changed = changed || temp_changed(this->target_humidity, this->last_target_humidity_);
    changed = changed || (this->mode != this->last_mode_);
    changed = changed || (this->action != this->last_action_);
    changed = changed || (this->fan_mode != this->last_fan_mode_);
    changed = changed || (this->preset != this->last_preset_);
    // custom_fan_mode_ is a const char* — pointer comparison is correct since
    // set_custom_fan_mode_() always resolves to the canonical pointer in traits.
    changed = changed || (this->get_custom_fan_mode().c_str() != this->last_custom_fan_mode_);
    if (!changed) {
      return;
    }
  }

  this->last_current_temp_ = this->current_temperature;
  this->last_current_humidity_ = this->current_humidity;
  this->last_target_low_ = this->target_temperature_low;
  this->last_target_high_ = this->target_temperature_high;
  this->last_target_humidity_ = this->target_humidity;
  this->last_mode_ = this->mode;
  this->last_action_ = this->action;
  this->last_fan_mode_ = this->fan_mode;
  this->last_preset_ = this->preset;
  this->last_custom_fan_mode_ = this->get_custom_fan_mode().c_str();
  this->has_published_ = true;
  this->publish_state();
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
