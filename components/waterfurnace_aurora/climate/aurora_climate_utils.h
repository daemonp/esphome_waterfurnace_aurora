#pragma once

// Shared climate mode mapping functions — used by both AuroraClimate and AuroraIZ2Climate.
// These eliminate the duplicated switch statements for HeatingMode↔ClimateMode
// and FanMode↔ClimateFanMode conversions.
//
// Ruby gem equivalent: bidirectional Hash lookups (HEATING_MODE[value] / HEATING_MODE.invert[symbol])

#include "esphome/components/climate/climate.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

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

// Convert Aurora FanMode to ESPHome ClimateFanMode
inline climate::ClimateFanMode aurora_to_esphome_fan(FanMode aurora_fan) {
  switch (aurora_fan) {
    case FanMode::AUTO:
      return climate::CLIMATE_FAN_AUTO;
    case FanMode::CONTINUOUS:
    case FanMode::INTERMITTENT:
      return climate::CLIMATE_FAN_ON;
    default:
      return climate::CLIMATE_FAN_AUTO;
  }
}

// Convert ESPHome ClimateFanMode to Aurora FanMode
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

}  // namespace waterfurnace_aurora
}  // namespace esphome
