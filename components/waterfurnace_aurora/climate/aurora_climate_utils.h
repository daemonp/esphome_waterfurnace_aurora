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
    case HEATING_MODE_OFF:
      mode = climate::CLIMATE_MODE_OFF;
      return true;
    case HEATING_MODE_AUTO:
      mode = climate::CLIMATE_MODE_HEAT_COOL;
      return true;
    case HEATING_MODE_COOL:
      mode = climate::CLIMATE_MODE_COOL;
      return true;
    case HEATING_MODE_HEAT:
      mode = climate::CLIMATE_MODE_HEAT;
      return true;
    case HEATING_MODE_EHEAT:
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
      aurora_mode = HEATING_MODE_OFF;
      return true;
    case climate::CLIMATE_MODE_HEAT_COOL:
      aurora_mode = HEATING_MODE_AUTO;
      return true;
    case climate::CLIMATE_MODE_COOL:
      aurora_mode = HEATING_MODE_COOL;
      return true;
    case climate::CLIMATE_MODE_HEAT:
      aurora_mode = HEATING_MODE_HEAT;
      return true;
    default:
      return false;
  }
}

// Convert Aurora FanMode to ESPHome ClimateFanMode
inline climate::ClimateFanMode aurora_to_esphome_fan(FanMode aurora_fan) {
  switch (aurora_fan) {
    case FAN_MODE_AUTO:
      return climate::CLIMATE_FAN_AUTO;
    case FAN_MODE_CONTINUOUS:
    case FAN_MODE_INTERMITTENT:
      return climate::CLIMATE_FAN_ON;
    default:
      return climate::CLIMATE_FAN_AUTO;
  }
}

// Convert ESPHome ClimateFanMode to Aurora FanMode
inline FanMode esphome_to_aurora_fan(climate::ClimateFanMode fan_mode) {
  switch (fan_mode) {
    case climate::CLIMATE_FAN_AUTO:
      return FAN_MODE_AUTO;
    case climate::CLIMATE_FAN_ON:
      return FAN_MODE_CONTINUOUS;
    default:
      return FAN_MODE_AUTO;
  }
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
