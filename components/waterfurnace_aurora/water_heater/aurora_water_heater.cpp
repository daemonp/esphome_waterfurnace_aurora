#include "aurora_water_heater.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include <cmath>

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.water_heater";

void AuroraWaterHeater::setup() {
  // Initialize flash preferences for state persistence across reboots.
  water_heater::WaterHeater::setup();

  // Register with parent to receive data update notifications.
  if (this->parent_ != nullptr) {
    this->parent_->register_listener([this]() { this->update_state_(); });
  }
}

void AuroraWaterHeater::dump_config() {
  LOG_WATER_HEATER("", "Aurora", this);
  this->dump_traits_(TAG);
  ESP_LOGCONFIG(TAG, "  Parent: %s", this->parent_ != nullptr ? "configured" : "NOT SET");
}

water_heater::WaterHeaterTraits AuroraWaterHeater::traits() {
  auto traits = water_heater::WaterHeaterTraits();

  // Feature flags
  traits.add_feature_flags(water_heater::WATER_HEATER_SUPPORTS_CURRENT_TEMPERATURE);
  traits.add_feature_flags(water_heater::WATER_HEATER_SUPPORTS_TARGET_TEMPERATURE);
  traits.add_feature_flags(water_heater::WATER_HEATER_SUPPORTS_OPERATION_MODE);

  // Supported modes: OFF (DHW disabled) and HEAT_PUMP (DHW enabled via geothermal)
  water_heater::WaterHeaterModeMask modes;
  modes.insert(water_heater::WATER_HEATER_MODE_OFF);
  modes.insert(water_heater::WATER_HEATER_MODE_HEAT_PUMP);
  traits.set_supported_modes(modes);

  // Temperature range: 100-140°F converted to °C for ESPHome
  // 100°F = 37.8°C, 140°F = 60.0°C
  traits.set_min_temperature(fahrenheit_to_celsius(100));
  traits.set_max_temperature(fahrenheit_to_celsius(140));
  traits.set_target_temperature_step(0.5);

  return traits;
}

water_heater::WaterHeaterCallInternal AuroraWaterHeater::make_call() {
  return water_heater::WaterHeaterCallInternal(this);
}

void AuroraWaterHeater::control(const water_heater::WaterHeaterCall &call) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control DHW");
    return;
  }

  // Handle mode change: OFF = DHW disabled, HEAT_PUMP = DHW enabled
  if (call.get_mode().has_value()) {
    water_heater::WaterHeaterMode mode = *call.get_mode();
    bool enable = (mode != water_heater::WATER_HEATER_MODE_OFF);
    if (this->parent_->set_dhw_enabled(enable)) {
      this->set_mode_(mode);
    }
  }

  // Handle target temperature change
  // HA sends °C; hardware expects °F
  float target = call.get_target_temperature();
  if (!std::isnan(target)) {
    float temp_f = celsius_to_fahrenheit(target);
    if (this->parent_->set_dhw_setpoint(temp_f)) {
      this->set_target_temperature_(target);
    }
  }

  this->publish_state();
}

void AuroraWaterHeater::update_state_() {
  if (this->parent_ == nullptr) {
    return;
  }

  // Update current temperature (hardware reports °F, water_heater entity uses °C)
  float dhw_temp = this->parent_->get_dhw_temperature();
  if (!std::isnan(dhw_temp)) {
    this->set_current_temperature(fahrenheit_to_celsius(dhw_temp));
  }

  // Update target temperature (skip during write cooldown)
  if (!this->parent_->dhw_cooldown_active()) {
    float dhw_setpoint = this->parent_->get_dhw_setpoint();
    if (!std::isnan(dhw_setpoint)) {
      this->set_target_temperature_(fahrenheit_to_celsius(dhw_setpoint));
    }

    // Update mode: DHW enabled → HEAT_PUMP, DHW disabled → OFF
    bool enabled = this->parent_->is_dhw_enabled();
    this->set_mode_(enabled ? water_heater::WATER_HEATER_MODE_HEAT_PUMP
                            : water_heater::WATER_HEATER_MODE_OFF);
  }

  this->publish_state();
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
