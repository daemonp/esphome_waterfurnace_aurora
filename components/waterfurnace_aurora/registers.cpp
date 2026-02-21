#include "registers.h"

namespace esphome {
namespace waterfurnace_aurora {

// ============================================================================
// Bitmask-to-String
// ============================================================================

std::string bitmask_to_string(uint16_t value, const BitLabel *bits, size_t count) {
  if (value == 0) return "None";

  std::string result;
  result.reserve(64);
  for (size_t i = 0; i < count; i++) {
    if (value & bits[i].mask) {
      if (!result.empty()) result += ", ";
      result += bits[i].label;
    }
  }
  return result.empty() ? "Unknown" : result;
}

// ============================================================================
// registers_to_string
// ============================================================================

std::string registers_to_string(const std::vector<uint16_t> &regs) {
  std::string result;
  result.reserve(regs.size() * 2);

  for (uint16_t reg : regs) {
    char high_char = static_cast<char>(reg >> 8);
    char low_char = static_cast<char>(reg & 0xFF);

    if (high_char == '\0') break;
    result += high_char;

    if (low_char == '\0') break;
    result += low_char;
  }

  // Strip trailing spaces/nulls (matches Ruby gem behavior)
  while (!result.empty() && (result.back() == ' ' || result.back() == '\0')) {
    result.pop_back();
  }

  return result;
}

// ============================================================================
// Fault Description Lookup
// ============================================================================

const char *get_fault_description(uint8_t code) {
  switch (code) {
    case 0: return "No Fault";
    case 1: return "Input Error";
    case 2: return "High Pressure";
    case 3: return "Low Pressure";
    case 4: return "Freeze Detect FP2";
    case 5: return "Freeze Detect FP1";
    case 7: return "Condensate Overflow";
    case 8: return "Over/Under Voltage";
    case 9: return "AirF/RPM";
    case 10: return "Compressor Monitor";
    case 11: return "FP1/2 Sensor Error";
    case 12: return "RefPerfrm Error";
    case 13: return "Non-Critical AXB Sensor Error";
    case 14: return "Critical AXB Sensor Error";
    case 15: return "Hot Water Limit";
    case 16: return "VS Pump Error";
    case 17: return "Communicating Thermostat Error";
    case 18: return "Non-Critical Communications Error";
    case 19: return "Critical Communications Error";
    case 21: return "Low Loop Pressure";
    case 22: return "Communicating ECM Error";
    case 23: return "HA Alarm 1";
    case 24: return "HA Alarm 2";
    case 25: return "AxbEev Error";
    case 41: return "High Drive Temp";
    case 42: return "High Discharge Temp";
    case 43: return "Low Suction Pressure";
    case 44: return "Low Condensing Pressure";
    case 45: return "High Condensing Pressure";
    case 46: return "Output Power Limit";
    case 47: return "EEV ID Comm Error";
    case 48: return "EEV OD Comm Error";
    case 49: return "Cabinet Temperature Sensor";
    case 51: return "Discharge Temp Sensor";
    case 52: return "Suction Pressure Sensor";
    case 53: return "Condensing Pressure Sensor";
    case 54: return "Low Supply Voltage";
    case 55: return "Out of Envelope";
    case 56: return "Drive Over Current";
    case 57: return "Drive Over/Under Voltage";
    case 58: return "High Drive Temp";
    case 59: return "Internal Drive Error";
    case 61: return "Multiple Safe Mode";
    case 71: return "Loss of Charge";
    case 72: return "Suction Temperature Sensor";
    case 73: return "Leaving Air Temperature Sensor";
    case 74: return "Maximum Operating Pressure";
    case 99: return "System Reset";
    default: return "Unknown Fault";
  }
}

// ============================================================================
// String Lookup Functions
// ============================================================================

const char *get_hvac_mode_string(HeatingMode mode) {
  switch (mode) {
    case HeatingMode::OFF: return "Off";
    case HeatingMode::AUTO: return "Auto";
    case HeatingMode::COOL: return "Cool";
    case HeatingMode::HEAT: return "Heat";
    case HeatingMode::EHEAT: return "Emergency Heat";
    default: return "Unknown";
  }
}

const char *get_fan_mode_string(FanMode mode) {
  switch (mode) {
    case FanMode::AUTO: return "Auto";
    case FanMode::CONTINUOUS: return "Continuous";
    case FanMode::INTERMITTENT: return "Intermittent";
    default: return "Unknown";
  }
}

const char *get_pump_type_string(PumpType type) {
  switch (type) {
    case PumpType::OPEN_LOOP: return "Open Loop";
    case PumpType::FC1: return "FC1";
    case PumpType::FC2: return "FC2";
    case PumpType::VS_PUMP: return "VS Pump";
    case PumpType::VS_PUMP_26_99: return "VS Pump + 26-99";
    case PumpType::VS_PUMP_UPS26_99: return "VS Pump + UPS26-99";
    case PumpType::FC1_GLNP: return "FC1_GLNP";
    case PumpType::FC2_GLNP: return "FC2_GLNP";
    default: return "Other";
  }
}

std::string get_vs_derate_string(uint16_t value) {
  return bitmask_to_string(value, VS_DERATE_BITS, VS_DERATE_BITS_COUNT);
}

std::string get_vs_safe_mode_string(uint16_t value) {
  return bitmask_to_string(value, VS_SAFE_MODE_BITS, VS_SAFE_MODE_BITS_COUNT);
}

std::string get_vs_alarm_string(uint16_t alarm1, uint16_t alarm2) {
  if (alarm1 == 0 && alarm2 == 0) return "None";

  std::string result;
  result.reserve(128);

  // Alarm1 flags
  if (alarm1 & 0x8000) {
    if (!result.empty()) result += ", ";
    result += "Internal Error";
  }

  // Alarm2 flags — use inline table since this one has many specific bits
  static constexpr BitLabel ALARM2_BITS[] = {
    {0x0001, "Multi Safe Modes"},
    {0x0002, "Out of Envelope"},
    {0x0004, "Over Current"},
    {0x0008, "Over Voltage"},
    {0x0010, "Drive Over Temp"},
    {0x0020, "Under Voltage"},
    {0x0040, "High Discharge Temp"},
    {0x0080, "Invalid Discharge Temp"},
    {0x0100, "OEM Comms Timeout"},
    {0x0200, "MOC Safety"},
    {0x0400, "DC Under Voltage"},
    {0x0800, "Invalid Suction Pressure"},
    {0x1000, "Invalid Discharge Pressure"},
    {0x2000, "Low Discharge Pressure"},
  };

  for (const auto &bit : ALARM2_BITS) {
    if (alarm2 & bit.mask) {
      if (!result.empty()) result += ", ";
      result += bit.label;
    }
  }

  return result.empty() ? "Unknown" : result;
}

std::string get_axb_inputs_string(uint16_t value) {
  std::string result;
  result.reserve(64);

  if (value & 0x001) {
    if (!result.empty()) result += ", ";
    result += "SmartGrid";
  }
  if (value & 0x002) {
    if (!result.empty()) result += ", ";
    result += "HA1";
  }
  if (value & 0x004) {
    if (!result.empty()) result += ", ";
    result += "HA2";
  }
  if (value & 0x008) {
    if (!result.empty()) result += ", ";
    result += "PumpSlave";
  }

  // MB Address: bit 4 determines address 3 or 4
  if (!result.empty()) result += ", ";
  result += "Addr=";
  result += (value & 0x010) ? "3" : "4";

  // Accessory relay 2 mode (bits 7-8)
  if (!result.empty()) result += ", ";
  result += "Acc2=";
  bool bit7 = (value & 0x080) != 0;
  bool bit8 = (value & 0x100) != 0;
  if (bit7 && bit8) {
    result += "Blower";
  } else if (bit8) {
    result += "LowCapComp";
  } else if (bit7) {
    result += "HighCapComp";
  } else {
    result += "Dehum";
  }

  return result;
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
