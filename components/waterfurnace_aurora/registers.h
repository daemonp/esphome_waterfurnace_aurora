#pragma once

// WaterFurnace Aurora Register Definitions Module
//
// Standalone, unit-testable module for register knowledge.
// NO ESPHome dependencies — pure C++ with standard library only.
//
// Contains: register addresses, enums, bitmask constants, fault table,
// data type conversions, IZ2 zone extraction helpers, and bitmask-to-string
// formatters. All protocol knowledge about what WaterFurnace registers mean
// lives here; the hub just reads/writes raw values.
//
// Improvements over rw-wf's registers.h (470 lines):
//   - RegisterType enum (compile-time) instead of runtime string comparison
//   - Sentinel-aware conversions built into convert_register()
//   - Constexpr BitLabel tables for bitmask-to-string (no heap per entry)
//   - IZ2 zone extraction as standalone inline functions (testable)
//   - RegisterCapability enum with zero-cost dispatch (not string→enum at setup)

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace esphome {
namespace waterfurnace_aurora {

// ============================================================================
// Register Type System
// ============================================================================

/// Data type of a register value. Used for converting raw uint16_t to float.
/// rw-wf stores this as std::string ("signed_tenths") and does runtime strcmp.
/// We use an enum — zero overhead at runtime.
enum class RegisterType : uint8_t {
  UNSIGNED,        // Raw uint16
  SIGNED,          // int16
  TENTHS,          // uint16 / 10.0
  SIGNED_TENTHS,   // int16 / 10.0 (sentinel-aware: -999.9/999.9 → NAN)
  HUNDREDTHS,      // uint16 / 100.0
  BOOLEAN,         // 0 or 1
  UINT32,          // Two consecutive registers: (hi << 16) | lo
  INT32,           // Two consecutive registers, signed
};

/// Hardware capability required to poll a register.
/// rw-wf passes this as a string from Python and converts at setup.
/// We use an enum directly — Python codegen emits the enum value.
enum class RegisterCapability : uint8_t {
  NONE,               // Always pollable
  AWL_THERMOSTAT,     // Thermostat v3.0+
  AWL_AXB,            // AXB v2.0+
  AWL_COMMUNICATING,  // Thermostat v3.0+ OR IZ2 v2.0+
  AXB,                // AXB present
  REFRIGERATION,      // AXB + energy_monitor >= 1
  ENERGY,             // AXB + energy_monitor == 2
  VS_DRIVE,           // VS drive present
  IZ2,                // IZ2 with AWL v2.0+
};

// ============================================================================
// Enums (from registers.rb)
// ============================================================================

enum class HeatingMode : uint8_t {
  OFF = 0,
  AUTO = 1,
  COOL = 2,
  HEAT = 3,
  EHEAT = 4
};

enum class FanMode : uint8_t {
  AUTO = 0,
  CONTINUOUS = 1,
  INTERMITTENT = 2
};

enum class BlowerType : uint8_t {
  PSC = 0,
  ECM_208 = 1,
  ECM_265 = 2,
  FIVE_SPEED = 3
};

enum class PumpType : uint8_t {
  OPEN_LOOP = 0,
  FC1 = 1,
  FC2 = 2,
  VS_PUMP = 3,
  VS_PUMP_26_99 = 4,
  VS_PUMP_UPS26_99 = 5,
  FC1_GLNP = 6,
  FC2_GLNP = 7,
  OTHER = 255
};

// IZ2 Zone current mode/call (from registers.rb CALLS hash)
enum class ZoneCall : uint8_t {
  STANDBY = 0,
  UNKNOWN1 = 1,
  H1 = 2,
  H2 = 3,
  H3 = 4,
  C1 = 5,
  C2 = 6,
  UNKNOWN7 = 7
};

enum class ZonePriority : uint8_t {
  COMFORT = 0,
  ECONOMY = 1
};

enum class ZoneSize : uint8_t {
  QUARTER = 0,
  HALF = 1,
  THREE_QUARTER = 2,
  FULL = 3
};

// Maximum number of IZ2 zones
static constexpr uint8_t MAX_IZ2_ZONES = 6;

// Component detection sentinel values — registers return these when the component is not installed.
// The ABC board uses 3 for "removed/not installed" and 0xFFFF for "not present/unsupported".
static constexpr uint16_t COMPONENT_NOT_INSTALLED = 3;
static constexpr uint16_t COMPONENT_UNSUPPORTED = 0xFFFF;

// ============================================================================
// Register Addresses (from registers.rb)
// ============================================================================

namespace registers {
  // System info
  static constexpr uint16_t ABC_VERSION = 2;
  static constexpr uint16_t COMPRESSOR_ANTI_SHORT_CYCLE = 6;
  static constexpr uint16_t LINE_VOLTAGE = 16;
  static constexpr uint16_t FP1_TEMP = 19;
  static constexpr uint16_t FP2_TEMP = 20;
  static constexpr uint16_t LAST_FAULT = 25;
  static constexpr uint16_t LAST_LOCKOUT_FAULT = 26;     // High bit = locked, bits 0-14 = fault code
  static constexpr uint16_t OUTPUTS_AT_LOCKOUT = 27;     // Bitmask: system outputs when lockout occurred
  static constexpr uint16_t INPUTS_AT_LOCKOUT = 28;      // Bitmask: system status/inputs when lockout occurred
  static constexpr uint16_t SYSTEM_OUTPUTS = 30;
  static constexpr uint16_t SYSTEM_STATUS = 31;
  static constexpr uint16_t ABC_PROGRAM = 88;         // 4 registers (88-91) — program version string
  static constexpr uint16_t MODEL_NUMBER = 92;        // 12 registers (92-103)
  static constexpr uint16_t SERIAL_NUMBER = 105;      // 5 registers (105-109)
  static constexpr uint16_t LINE_VOLTAGE_SETTING = 112;

  // VS Drive details
  static constexpr uint16_t VS_DERATE = 214;
  static constexpr uint16_t VS_SAFE_MODE = 216;
  static constexpr uint16_t VS_ALARM1 = 217;
  static constexpr uint16_t VS_ALARM2 = 218;

  // Blower / ECM
  static constexpr uint16_t VS_PUMP_MIN = 321;
  static constexpr uint16_t VS_PUMP_MAX = 322;
  static constexpr uint16_t VS_PUMP_MANUAL = 323;
  static constexpr uint16_t VS_PUMP_SPEED = 325;
  static constexpr uint16_t BLOWER_ONLY_SPEED = 340;
  static constexpr uint16_t LO_COMPRESSOR_ECM_SPEED = 341;
  static constexpr uint16_t HI_COMPRESSOR_ECM_SPEED = 342;
  static constexpr uint16_t ECM_SPEED = 344;
  static constexpr uint16_t AUX_HEAT_ECM_SPEED = 347;
  static constexpr uint16_t ACTIVE_DEHUMIDIFY = 362;

  // DHW
  static constexpr uint16_t DHW_ENABLED = 400;
  static constexpr uint16_t DHW_SETPOINT = 401;

  // Hardware detection
  static constexpr uint16_t BLOWER_TYPE = 404;
  static constexpr uint16_t ENERGY_MONITOR = 412;
  static constexpr uint16_t PUMP_TYPE = 413;

  // IZ2 system registers
  static constexpr uint16_t IZ2_NUM_ZONES = 483;
  static constexpr uint16_t AMBIENT_TEMP = 502;
  static constexpr uint16_t IZ2_COMPRESSOR_SPEED_DESIRED = 564;
  static constexpr uint16_t IZ2_BLOWER_SPEED_DESIRED = 565;
  static constexpr uint16_t ENTERING_AIR = 567;

  // Fault history
  static constexpr uint16_t FAULT_HISTORY_START = 601;
  static constexpr uint16_t FAULT_HISTORY_END = 699;

  // Thermostat
  static constexpr uint16_t ENTERING_AIR_AWL = 740;
  static constexpr uint16_t RELATIVE_HUMIDITY = 741;
  static constexpr uint16_t OUTDOOR_TEMP = 742;
  static constexpr uint16_t HEATING_SETPOINT = 745;
  static constexpr uint16_t COOLING_SETPOINT = 746;
  static constexpr uint16_t THERMOSTAT_INSTALLED = 800;
  static constexpr uint16_t THERMOSTAT_VERSION = 801;
  static constexpr uint16_t AXB_INSTALLED = 806;
  static constexpr uint16_t AXB_VERSION = 807;
  static constexpr uint16_t IZ2_INSTALLED = 812;
  static constexpr uint16_t IZ2_VERSION = 813;
  static constexpr uint16_t LEAVING_AIR = 900;

  // AXB
  static constexpr uint16_t AXB_INPUTS = 1103;
  static constexpr uint16_t AXB_OUTPUTS = 1104;
  static constexpr uint16_t HEATING_LIQUID_LINE_TEMP = 1109;
  static constexpr uint16_t LEAVING_WATER = 1110;
  static constexpr uint16_t ENTERING_WATER = 1111;
  static constexpr uint16_t DHW_TEMP = 1114;
  static constexpr uint16_t DISCHARGE_PRESSURE = 1115;
  static constexpr uint16_t SUCTION_PRESSURE = 1116;
  static constexpr uint16_t WATERFLOW = 1117;
  static constexpr uint16_t LOOP_PRESSURE = 1119;
  static constexpr uint16_t SATURATED_CONDENSER_TEMP = 1134;
  static constexpr uint16_t SUBCOOL_HEATING = 1135;
  static constexpr uint16_t SUBCOOL_COOLING = 1136;

  // Energy monitoring (32-bit values, high word first)
  static constexpr uint16_t COMPRESSOR_WATTS = 1146;
  static constexpr uint16_t BLOWER_WATTS = 1148;
  static constexpr uint16_t AUX_WATTS = 1150;
  static constexpr uint16_t TOTAL_WATTS = 1152;
  static constexpr uint16_t HEAT_OF_EXTRACTION = 1154;
  static constexpr uint16_t HEAT_OF_REJECTION = 1156;
  static constexpr uint16_t PUMP_WATTS = 1164;

  // VS Drive
  static constexpr uint16_t COMPRESSOR_SPEED_DESIRED = 3000;
  static constexpr uint16_t COMPRESSOR_SPEED_ACTUAL = 3001;
  static constexpr uint16_t VS_DISCHARGE_PRESSURE = 3322;
  static constexpr uint16_t VS_SUCTION_PRESSURE = 3323;
  static constexpr uint16_t VS_DISCHARGE_TEMP = 3325;
  static constexpr uint16_t VS_AMBIENT_TEMP = 3326;
  static constexpr uint16_t VS_DRIVE_TEMP = 3327;
  static constexpr uint16_t VS_COMPRESSOR_WATTS = 3422;
  static constexpr uint16_t VS_INVERTER_TEMP = 3522;
  static constexpr uint16_t VS_FAN_SPEED = 3524;
  static constexpr uint16_t VS_EEV_OPEN = 3808;
  static constexpr uint16_t VS_SUCTION_TEMP = 3903;
  static constexpr uint16_t VS_SAT_EVAP_DISCHARGE_TEMP = 3905;
  static constexpr uint16_t VS_SUPERHEAT_TEMP = 3906;

  // Thermostat config (read)
  static constexpr uint16_t FAN_CONFIG = 12005;
  static constexpr uint16_t HEATING_MODE_READ = 12006;

  // System commands
  static constexpr uint16_t CLEAR_FAULT_HISTORY = 47;
  static constexpr uint16_t CLEAR_FAULT_MAGIC = 0x5555;

  // Humidistat
  static constexpr uint16_t HUMIDISTAT_SETTINGS = 12309;
  static constexpr uint16_t HUMIDISTAT_TARGETS = 12310;
  static constexpr uint16_t IZ2_HUMIDISTAT_SETTINGS = 21114;
  static constexpr uint16_t IZ2_HUMIDISTAT_TARGETS_WRITE = 21115;
  static constexpr uint16_t IZ2_HUMIDISTAT_MODE = 31109;
  static constexpr uint16_t IZ2_HUMIDISTAT_TARGETS = 31110;

  // Thermostat config (write)
  static constexpr uint16_t HEATING_MODE_WRITE = 12606;
  static constexpr uint16_t HEATING_SETPOINT_WRITE = 12619;
  static constexpr uint16_t COOLING_SETPOINT_WRITE = 12620;
  static constexpr uint16_t FAN_MODE_WRITE = 12621;
  static constexpr uint16_t FAN_INTERMITTENT_ON_WRITE = 12622;
  static constexpr uint16_t FAN_INTERMITTENT_OFF_WRITE = 12623;

  // IZ2 Zone registers (base addresses, add (zone-1)*offset for each zone)
  static constexpr uint16_t IZ2_MODE_WRITE_BASE = 21202;
  static constexpr uint16_t IZ2_HEAT_SP_WRITE_BASE = 21203;
  static constexpr uint16_t IZ2_COOL_SP_WRITE_BASE = 21204;
  static constexpr uint16_t IZ2_FAN_MODE_WRITE_BASE = 21205;
  static constexpr uint16_t IZ2_FAN_ON_WRITE_BASE = 21206;
  static constexpr uint16_t IZ2_FAN_OFF_WRITE_BASE = 21207;
  static constexpr uint16_t IZ2_OUTDOOR_TEMP = 31003;
  static constexpr uint16_t IZ2_DEMAND = 31005;
  static constexpr uint16_t IZ2_AMBIENT_BASE = 31007;
  static constexpr uint16_t IZ2_CONFIG1_BASE = 31008;
  static constexpr uint16_t IZ2_CONFIG2_BASE = 31009;
  static constexpr uint16_t IZ2_CONFIG3_BASE = 31200;
}  // namespace registers

// ============================================================================
// System Output/Input Bitmasks (register 30, 31, 1103, 1104)
// ============================================================================

// System outputs (register 30)
static constexpr uint16_t OUTPUT_CC = 0x01;
static constexpr uint16_t OUTPUT_CC2 = 0x02;
static constexpr uint16_t OUTPUT_RV = 0x04;       // Reversing valve (cool mode)
static constexpr uint16_t OUTPUT_BLOWER = 0x08;
static constexpr uint16_t OUTPUT_EH1 = 0x10;      // Aux heat 1
static constexpr uint16_t OUTPUT_EH2 = 0x20;      // Aux heat 2
static constexpr uint16_t OUTPUT_ACCESSORY = 0x200;
static constexpr uint16_t OUTPUT_LOCKOUT = 0x400;
static constexpr uint16_t OUTPUT_ALARM = 0x800;

// AXB outputs (register 1104)
static constexpr uint16_t AXB_OUTPUT_DHW = 0x01;
static constexpr uint16_t AXB_OUTPUT_LOOP_PUMP = 0x02;
static constexpr uint16_t AXB_OUTPUT_DIVERTING_VALVE = 0x04;

// System status (register 31)
static constexpr uint16_t STATUS_LPS = 0x80;
static constexpr uint16_t STATUS_HPS = 0x100;
static constexpr uint16_t STATUS_Y1 = 0x01;
static constexpr uint16_t STATUS_Y2 = 0x02;
static constexpr uint16_t STATUS_W = 0x04;
static constexpr uint16_t STATUS_O = 0x08;
static constexpr uint16_t STATUS_G = 0x10;
static constexpr uint16_t STATUS_DH_RH = 0x20;
static constexpr uint16_t STATUS_EMERGENCY_SHUTDOWN = 0x40;
static constexpr uint16_t STATUS_LOAD_SHED = 0x200;

// VS Drive Derate flags (registers 214, 3223)
static constexpr uint16_t VS_DERATE_DRIVE_OVER_TEMP = 0x01;
static constexpr uint16_t VS_DERATE_LOW_SUCTION_PRESSURE = 0x04;
static constexpr uint16_t VS_DERATE_LOW_DISCHARGE_PRESSURE = 0x10;
static constexpr uint16_t VS_DERATE_HIGH_DISCHARGE_PRESSURE = 0x20;
static constexpr uint16_t VS_DERATE_OUTPUT_POWER_LIMIT = 0x40;

// VS Drive Safe Mode flags (registers 216, 3225)
static constexpr uint16_t VS_SAFE_EEV_INDOOR_FAILED = 0x01;
static constexpr uint16_t VS_SAFE_EEV_OUTDOOR_FAILED = 0x02;
static constexpr uint16_t VS_SAFE_INVALID_AMBIENT_TEMP = 0x04;

// ============================================================================
// Bitmask-to-String Helpers
// ============================================================================

struct BitLabel {
  uint16_t mask;
  const char *label;
};

/// Format a bitmask value as a comma-separated string of active flags.
/// Returns "None" if no flags are set.
std::string bitmask_to_string(uint16_t value, const BitLabel *bits, size_t count);

// Predefined bit label tables — defined in registers.cpp to avoid per-TU copies.
extern const BitLabel OUTPUT_BITS[];
inline constexpr size_t OUTPUT_BITS_COUNT = 9;

extern const BitLabel INPUT_BITS[];
inline constexpr size_t INPUT_BITS_COUNT = 10;

extern const BitLabel VS_DERATE_BITS[];
inline constexpr size_t VS_DERATE_BITS_COUNT = 5;

extern const BitLabel VS_SAFE_MODE_BITS[];
inline constexpr size_t VS_SAFE_MODE_BITS_COUNT = 3;

// ============================================================================
// Data Conversion Functions
// ============================================================================

/// Convert raw uint16_t register value to float based on RegisterType.
/// Sentinel values (-999.9 / 999.9) are converted to NAN automatically.
/// rw-wf does sentinel detection per-entity; we do it once here.
inline float convert_register(uint16_t raw, RegisterType type) {
  switch (type) {
    case RegisterType::UNSIGNED:
      return static_cast<float>(raw);

    case RegisterType::SIGNED:
      return static_cast<float>(static_cast<int16_t>(raw));

    case RegisterType::TENTHS:
      // Sentinel: 999.9 (raw 0x270F) → NAN
      if (raw == 0x270F) return NAN;
      return raw / 10.0f;

    case RegisterType::SIGNED_TENTHS: {
      // Sentinel: -999.9 (raw 0xD8F1) or 999.9 (raw 0x270F) → NAN
      if (raw == 0xD8F1 || raw == 0x270F) return NAN;
      return static_cast<int16_t>(raw) / 10.0f;
    }

    case RegisterType::HUNDREDTHS:
      return raw / 100.0f;

    case RegisterType::BOOLEAN:
      return (raw != 0) ? 1.0f : 0.0f;

    default:
      return static_cast<float>(raw);
  }
}

/// Convert raw signed tenths (int16 / 10.0). Standalone convenience function.
/// Sentinel values (-999.9 = 0xD8F1, 999.9 = 0x270F) map to NAN.
inline float to_signed_tenths(uint16_t value) {
  if (value == 0xD8F1 || value == 0x270F) return NAN;
  return static_cast<int16_t>(value) / 10.0f;
}

/// Convert raw unsigned tenths (uint16 / 10.0). Standalone convenience function.
/// Sentinel value (999.9 = 0x270F) maps to NAN.
inline float to_tenths(uint16_t value) {
  if (value == 0x270F) return NAN;
  return value / 10.0f;
}

/// Assemble two consecutive 16-bit registers into uint32 (high word first).
inline uint32_t to_uint32(uint16_t high, uint16_t low) {
  return (static_cast<uint32_t>(high) << 16) | low;
}

/// Assemble two consecutive 16-bit registers into int32 (high word first).
inline int32_t to_int32(uint16_t high, uint16_t low) {
  return static_cast<int32_t>(to_uint32(high, low));
}

/// Convert register values to ASCII string. Each register holds
/// two characters (high byte, low byte). Strips trailing spaces/nulls.
std::string registers_to_string(const std::vector<uint16_t> &regs);

/// Overload accepting a raw pointer + count (avoids heap allocation from vector construction).
std::string registers_to_string(const uint16_t *regs, size_t count);

// ============================================================================
// String Lookup Functions
// ============================================================================

/// Get fault description from fault code (67 known codes from registers.rb FAULTS).
/// Accepts uint16_t to avoid implicit narrowing from register values.
const char *get_fault_description(uint16_t code);

/// Get HVAC mode string.
const char *get_hvac_mode_string(HeatingMode mode);

/// Get fan mode string.
const char *get_fan_mode_string(FanMode mode);

/// Get pump type string.
const char *get_pump_type_string(PumpType type);

/// Get VS derate status string from bitmask.
std::string get_vs_derate_string(uint16_t value);

/// Get VS safe mode status string from bitmask.
std::string get_vs_safe_mode_string(uint16_t value);

/// Get VS alarm string from two alarm registers.
std::string get_vs_alarm_string(uint16_t alarm1, uint16_t alarm2);

/// Get AXB inputs string from bitmask.
std::string get_axb_inputs_string(uint16_t value);

/// Get system outputs string from bitmask (for lockout diagnostics).
std::string get_outputs_string(uint16_t value);

/// Get system inputs/status string from bitmask (for lockout diagnostics).
std::string get_inputs_string(uint16_t value);

// ============================================================================
// IZ2 Zone Extraction Helpers
// ============================================================================
//
// Extract zone configuration from packed IZ2 config registers.
// These are pure functions operating on raw register values — no hub state needed.
// Matches Ruby gem registers.rb IZ2 parsing (with our 3-bit mode fix for E-Heat).

/// Extract target mode from config2 register. Uses 3-bit mask (our fix over Ruby gem's 2-bit).
inline HeatingMode iz2_extract_mode(uint16_t config2) {
  uint8_t mode_val = (config2 >> 8) & 0x07;  // 3 bits to support EHEAT (4)
  if (mode_val > 4) mode_val = 0;
  return static_cast<HeatingMode>(mode_val);
}

/// Extract fan mode from config1 register.
inline FanMode iz2_extract_fan_mode(uint16_t config1) {
  if (config1 & 0x80) return FanMode::CONTINUOUS;
  if (config1 & 0x100) return FanMode::INTERMITTENT;
  return FanMode::AUTO;
}

/// Extract cooling setpoint from config1 register (°F).
inline float iz2_extract_cooling_setpoint(uint16_t config1) {
  return ((config1 & 0x7e) >> 1) + 36.0f;
}

/// Extract heating setpoint from config1 + config2 registers (°F).
inline float iz2_extract_heating_setpoint(uint16_t config1, uint16_t config2) {
  uint8_t carry = config1 & 0x01;
  return ((carry << 5) | ((config2 & 0xf800) >> 11)) + 36.0f;
}

/// Extract damper open state from config2 register.
inline bool iz2_extract_damper_open(uint16_t config2) {
  return (config2 & 0x10) != 0;
}

/// Extract current call (zone demand) from config2 register.
inline ZoneCall iz2_extract_current_call(uint16_t config2) {
  uint8_t call_val = (config2 >> 1) & 0x7;
  return static_cast<ZoneCall>(call_val);
}

/// Extract fan on time from config1 register (minutes, multiples of 5).
inline uint8_t iz2_extract_fan_on_time(uint16_t config1) {
  return ((config1 >> 9) & 0x7) * 5;
}

/// Extract fan off time from config1 register (minutes, multiples of 5).
inline uint8_t iz2_extract_fan_off_time(uint16_t config1) {
  return (((config1 >> 12) & 0x7) + 1) * 5;
}

/// Extract zone priority from config3 register.
inline ZonePriority iz2_extract_priority(uint16_t config3) {
  return (config3 & 0x20) ? ZonePriority::ECONOMY : ZonePriority::COMFORT;
}

/// Extract zone size from config3 register.
inline ZoneSize iz2_extract_size(uint16_t config3) {
  uint8_t size_val = (config3 >> 3) & 0x3;
  return static_cast<ZoneSize>(size_val);
}

/// Extract normalized zone size from config3 register.
inline uint8_t iz2_extract_normalized_size(uint16_t config3) {
  return (config3 >> 8) & 0xFF;
}

/// Convert IZ2 blower speed code (1-6) to percentage (25-100%).
/// From registers.rb iz2_fan_desired method.
inline uint8_t iz2_fan_desired(uint16_t value) {
  switch (value) {
    case 1: return 25;
    case 2: return 40;
    case 3: return 55;
    case 4: return 70;
    case 5: return 85;
    case 6: return 100;
    default: return static_cast<uint8_t>(value);
  }
}

// ============================================================================
// IZ2 Zone Data Structure
// ============================================================================

struct IZ2ZoneData {
  float ambient_temperature{NAN};
  float heating_setpoint{NAN};
  float cooling_setpoint{NAN};
  HeatingMode target_mode{HeatingMode::OFF};
  FanMode target_fan_mode{FanMode::AUTO};
  ZoneCall current_call{ZoneCall::STANDBY};
  bool damper_open{false};
  uint8_t fan_on_time{0};
  uint8_t fan_off_time{0};
  ZonePriority priority{ZonePriority::COMFORT};
  ZoneSize size{ZoneSize::FULL};
  uint8_t normalized_size{0};
};

// ============================================================================
// Flat Sorted Vector Register Map
// ============================================================================

// Eliminates per-element heap overhead of std::map.
// With ~90 registers, this saves ~4KB of fragmented heap vs std::map's tree nodes.
// Lookup is O(log n) via std::lower_bound on a contiguous array (cache-friendly).
using RegisterMap = std::vector<std::pair<uint16_t, uint16_t>>;

/// Find a register value in a sorted RegisterMap. Returns pointer to value or nullptr.
inline const uint16_t *reg_find(const RegisterMap &map, uint16_t addr) {
  auto it = std::lower_bound(map.begin(), map.end(), addr,
      [](const std::pair<uint16_t, uint16_t> &p, uint16_t a) { return p.first < a; });
  if (it != map.end() && it->first == addr)
    return &it->second;
  return nullptr;
}

/// Insert or update a register value in a sorted RegisterMap.
inline void reg_insert(RegisterMap &map, uint16_t addr, uint16_t value) {
  auto it = std::lower_bound(map.begin(), map.end(), addr,
      [](const std::pair<uint16_t, uint16_t> &p, uint16_t a) { return p.first < a; });
  if (it != map.end() && it->first == addr) {
    it->second = value;
  } else {
    map.insert(it, {addr, value});
  }
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
