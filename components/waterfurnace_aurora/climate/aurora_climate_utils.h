#pragma once

// Shared climate mode mapping functions — used by both AuroraClimate and AuroraIZ2Climate.
// These eliminate the duplicated switch statements for HeatingMode↔ClimateMode
// and FanMode↔ClimateFanMode conversions.
//
// Ruby gem equivalent: bidirectional Hash lookups (HEATING_MODE[value] / HEATING_MODE.invert[symbol])

#include "esphome/components/climate/climate.h"
#include "../waterfurnace_aurora.h"

#include <cmath>

namespace esphome {
namespace waterfurnace_aurora {

// Custom fan mode string for Intermittent (matches WaterFurnace thermostat UI)
// ESPHome's built-in ClimateFanMode enum has no "Intermittent" value, so we use
// the custom fan mode mechanism which allows arbitrary string labels.
// inline constexpr avoids per-TU duplication (vs. static const in a header).
inline constexpr const char *CUSTOM_FAN_MODE_INTERMITTENT = "Intermittent";

// Convert Aurora HeatingMode to ESPHome ClimateMode
// Returns false if the mode is not mappable (caller should handle)
inline bool aurora_to_esphome_mode(HeatingMode aurora_mode,
                                    climate::ClimateMode &mode,
                                    climate::ClimatePreset &preset) {
  preset = climate::CLIMATE_PRESET_NONE;
  switch (aurora_mode) {
    case HeatingMode::OFF:
      mode = climate::CLIMATE_MODE_OFF;
      return true;
    case HeatingMode::AUTO:
      mode = climate::CLIMATE_MODE_HEAT_COOL;
      return true;
    case HeatingMode::COOL:
      mode = climate::CLIMATE_MODE_COOL;
      return true;
    case HeatingMode::HEAT:
      mode = climate::CLIMATE_MODE_HEAT;
      return true;
    case HeatingMode::EHEAT:
      mode = climate::CLIMATE_MODE_HEAT;
      preset = climate::CLIMATE_PRESET_BOOST;  // E-Heat shown as BOOST preset
      return true;
    default:
      return false;
  }
}

// Convert ESPHome ClimateMode to Aurora HeatingMode
// Returns false if the mode is not mappable
inline bool esphome_to_aurora_mode(climate::ClimateMode mode, HeatingMode &aurora_mode) {
  switch (mode) {
    case climate::CLIMATE_MODE_OFF:
      aurora_mode = HeatingMode::OFF;
      return true;
    case climate::CLIMATE_MODE_HEAT_COOL:
      aurora_mode = HeatingMode::AUTO;
      return true;
    case climate::CLIMATE_MODE_COOL:
      aurora_mode = HeatingMode::COOL;
      return true;
    case climate::CLIMATE_MODE_HEAT:
      aurora_mode = HeatingMode::HEAT;
      return true;
    default:
      return false;
  }
}

// Convert Aurora FanMode to ESPHome ClimateFanMode for built-in modes.
// Returns the built-in fan mode, or nullopt for modes that need custom handling (Intermittent).
// Callers should check for INTERMITTENT separately and use set_custom_fan_mode_() from within
// the climate class (it's a protected method).
inline optional<climate::ClimateFanMode> aurora_to_esphome_fan(FanMode aurora_fan) {
  switch (aurora_fan) {
    case FanMode::AUTO:
      return climate::CLIMATE_FAN_AUTO;
    case FanMode::CONTINUOUS:
      return climate::CLIMATE_FAN_ON;
    case FanMode::INTERMITTENT:
    default:
      return {};  // Needs custom handling or unknown — caller decides
  }
}

// Convert ESPHome ClimateFanMode (built-in) to Aurora FanMode
inline FanMode esphome_to_aurora_fan(climate::ClimateFanMode fan_mode) {
  switch (fan_mode) {
    case climate::CLIMATE_FAN_AUTO:
      return FanMode::AUTO;
    case climate::CLIMATE_FAN_ON:
      return FanMode::CONTINUOUS;
    default:
      return FanMode::AUTO;
  }
}

// ============================================================================
// Humidity target helpers — shared by AuroraClimate and AuroraIZ2Climate
// ============================================================================

// TAG for humidity helper logging — inline constexpr avoids per-TU duplication.
inline constexpr const char *TAG_CLIMATE_UTILS = "aurora.climate";

/// Route a target_humidity write to the correct humidistat register based on HVAC mode.
/// Clamps to the mode-appropriate sub-range (15-50% for humidification, 35-65% for
/// dehumidification) with proper rounding. Returns true and sets out_target on success.
/// log_prefix is prepended to warning messages (e.g., "Zone 2: ").
inline bool route_humidity_write(WaterFurnaceAurora *hub, climate::ClimateMode mode,
                                 float target, float &out_target, const char *log_prefix) {
  switch (mode) {
    case climate::CLIMATE_MODE_HEAT:
    case climate::CLIMATE_MODE_HEAT_COOL: {
      float clamped_f = std::max(15.0f, std::min(50.0f, std::round(target)));
      uint8_t clamped = static_cast<uint8_t>(clamped_f);
      if (std::round(target) != clamped_f) {
        ESP_LOGW(TAG_CLIMATE_UTILS, "%sHumidification target clamped from %.0f to %u%%",
                 log_prefix, target, clamped);
      }
      if (hub->set_humidification_target(clamped)) {
        out_target = clamped_f;
        return true;
      }
      return false;
    }
    case climate::CLIMATE_MODE_COOL: {
      float clamped_f = std::max(35.0f, std::min(65.0f, std::round(target)));
      uint8_t clamped = static_cast<uint8_t>(clamped_f);
      if (std::round(target) != clamped_f) {
        ESP_LOGW(TAG_CLIMATE_UTILS, "%sDehumidification target clamped from %.0f to %u%%",
                 log_prefix, target, clamped);
      }
      if (hub->set_dehumidification_target(clamped)) {
        out_target = clamped_f;
        return true;
      }
      return false;
    }
    default:
      ESP_LOGW(TAG_CLIMATE_UTILS, "%sTarget humidity not applicable in current mode; ignoring",
               log_prefix);
      return false;
  }
}

/// Read the mode-appropriate humidistat target from the register cache.
/// Returns the target value (humidification for heat modes, dehumidification for cool),
/// or NAN if the register is not cached.
inline float read_mode_humidity_target(WaterFurnaceAurora *hub, climate::ClimateMode mode) {
  uint16_t target_reg = (hub->has_iz2() && hub->awl_communicating())
                            ? registers::IZ2_HUMIDISTAT_TARGETS
                            : registers::HUMIDISTAT_TARGETS;
  float raw = hub->get_cached_register(target_reg);
  if (std::isnan(raw)) return NAN;

  uint16_t packed = static_cast<uint16_t>(raw);
  switch (mode) {
    case climate::CLIMATE_MODE_COOL:
      return static_cast<float>(packed & 0xFF);          // Low byte: dehumidification
    case climate::CLIMATE_MODE_HEAT:
    case climate::CLIMATE_MODE_HEAT_COOL:
    default:
      return static_cast<float>((packed >> 8) & 0xFF);   // High byte: humidification
  }
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
