#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <vector>
#include <algorithm>
#include <functional>
#include <cmath>

namespace esphome {
namespace waterfurnace_aurora {

// WaterFurnace Aurora uses custom Modbus function codes:
// Function Code 0x41 ('A') - Read multiple register ranges (address + length pairs)
// Function Code 0x42 ('B') - Read specific individual registers (list of addresses)
// Function Code 0x03 - Standard read holding registers (also supported)
// Function Code 0x06 - Standard write single register

// Heating modes (from registers.rb HEATING_MODE)
enum class HeatingMode : uint8_t {
  OFF = 0,
  AUTO = 1,
  COOL = 2,
  HEAT = 3,
  EHEAT = 4
};

// Fan modes (from registers.rb FAN_MODE)
enum class FanMode : uint8_t {
  AUTO = 0,
  CONTINUOUS = 1,
  INTERMITTENT = 2
};

// System outputs bitmask (register 30, from registers.rb SYSTEM_OUTPUTS)
constexpr uint16_t OUTPUT_CC = 0x01;       // Compressor stage 1
constexpr uint16_t OUTPUT_CC2 = 0x02;      // Compressor stage 2
constexpr uint16_t OUTPUT_RV = 0x04;       // Reversing valve (cool instead of heat)
constexpr uint16_t OUTPUT_BLOWER = 0x08;
constexpr uint16_t OUTPUT_EH1 = 0x10;      // Aux heat 1
constexpr uint16_t OUTPUT_EH2 = 0x20;      // Aux heat 2
constexpr uint16_t OUTPUT_ACCESSORY = 0x200;
constexpr uint16_t OUTPUT_LOCKOUT = 0x400;
constexpr uint16_t OUTPUT_ALARM = 0x800;

// AXB outputs bitmask (register 1104, from registers.rb AXB_OUTPUTS)
constexpr uint16_t AXB_OUTPUT_DHW = 0x01;
constexpr uint16_t AXB_OUTPUT_LOOP_PUMP = 0x02;
constexpr uint16_t AXB_OUTPUT_DIVERTING_VALVE = 0x04;

// System status bitmask (register 31, from registers.rb status method)
constexpr uint16_t STATUS_LPS = 0x80;          // Low pressure switch (bit 7)
constexpr uint16_t STATUS_HPS = 0x100;         // High pressure switch (bit 8)
constexpr uint16_t STATUS_Y1 = 0x01;
constexpr uint16_t STATUS_Y2 = 0x02;
constexpr uint16_t STATUS_W = 0x04;
constexpr uint16_t STATUS_O = 0x08;
constexpr uint16_t STATUS_G = 0x10;
constexpr uint16_t STATUS_DH_RH = 0x20;
constexpr uint16_t STATUS_EMERGENCY_SHUTDOWN = 0x40;
constexpr uint16_t STATUS_LOAD_SHED = 0x200;

// VS Drive Derate flags (registers 214, 3223)
constexpr uint16_t VS_DERATE_DRIVE_OVER_TEMP = 0x01;
constexpr uint16_t VS_DERATE_LOW_SUCTION_PRESSURE = 0x04;
constexpr uint16_t VS_DERATE_LOW_DISCHARGE_PRESSURE = 0x10;
constexpr uint16_t VS_DERATE_HIGH_DISCHARGE_PRESSURE = 0x20;
constexpr uint16_t VS_DERATE_OUTPUT_POWER_LIMIT = 0x40;

// VS Drive Safe Mode flags (registers 216, 3225)
constexpr uint16_t VS_SAFE_EEV_INDOOR_FAILED = 0x01;
constexpr uint16_t VS_SAFE_EEV_OUTDOOR_FAILED = 0x02;
constexpr uint16_t VS_SAFE_INVALID_AMBIENT_TEMP = 0x04;

// Key register addresses (from registers.rb)
namespace registers {
  // System info
  constexpr uint16_t ABC_VERSION = 2;
  constexpr uint16_t COMPRESSOR_ANTI_SHORT_CYCLE = 6;
  constexpr uint16_t LINE_VOLTAGE = 16;
  constexpr uint16_t FP1_TEMP = 19;           // Cooling liquid line temp
  constexpr uint16_t FP2_TEMP = 20;           // Air coil temp
  constexpr uint16_t LAST_FAULT = 25;
  constexpr uint16_t SYSTEM_OUTPUTS = 30;
  constexpr uint16_t SYSTEM_STATUS = 31;
  constexpr uint16_t MODEL_NUMBER = 92;       // 12 registers (92-103)
  constexpr uint16_t SERIAL_NUMBER = 105;     // 5 registers (105-109)
  constexpr uint16_t LINE_VOLTAGE_SETTING = 112;
  
  // VS Drive details (from registers.rb)
  constexpr uint16_t VS_DERATE = 214;         // Also at 3223
  constexpr uint16_t VS_SAFE_MODE = 216;      // Also at 3225
  constexpr uint16_t VS_ALARM1 = 217;         // Also at 3226
  constexpr uint16_t VS_ALARM2 = 218;         // Also at 3227
  
  // Fault history (registers 601-699)
  constexpr uint16_t FAULT_HISTORY_START = 601;
  constexpr uint16_t FAULT_HISTORY_END = 699;
  
  // DHW (requires AXB)
  constexpr uint16_t DHW_ENABLED = 400;
  constexpr uint16_t DHW_SETPOINT = 401;
  
  // Thermostat
  constexpr uint16_t AMBIENT_TEMP = 502;
  constexpr uint16_t ENTERING_AIR = 567;
  constexpr uint16_t ENTERING_AIR_AWL = 740;
  constexpr uint16_t RELATIVE_HUMIDITY = 741;
  constexpr uint16_t OUTDOOR_TEMP = 742;
  constexpr uint16_t HEATING_SETPOINT = 745;
  constexpr uint16_t COOLING_SETPOINT = 746;
  constexpr uint16_t THERMOSTAT_INSTALLED = 800;
  constexpr uint16_t THERMOSTAT_VERSION = 801;
  constexpr uint16_t AXB_INSTALLED = 806;
  constexpr uint16_t AXB_VERSION = 807;
  constexpr uint16_t LEAVING_AIR = 900;
  
  // AXB
  constexpr uint16_t AXB_INPUTS = 1103;
  constexpr uint16_t AXB_OUTPUTS = 1104;
  constexpr uint16_t LEAVING_WATER = 1110;
  constexpr uint16_t ENTERING_WATER = 1111;
  constexpr uint16_t DHW_TEMP = 1114;
  constexpr uint16_t DISCHARGE_PRESSURE = 1115;
  constexpr uint16_t SUCTION_PRESSURE = 1116;
  constexpr uint16_t WATERFLOW = 1117;
  constexpr uint16_t LOOP_PRESSURE = 1119;
  
  // Blower / ECM (from blower.rb)
  constexpr uint16_t BLOWER_ONLY_SPEED = 340;
  constexpr uint16_t LO_COMPRESSOR_ECM_SPEED = 341;
  constexpr uint16_t HI_COMPRESSOR_ECM_SPEED = 342;
  constexpr uint16_t ECM_SPEED = 344;
  constexpr uint16_t AUX_HEAT_ECM_SPEED = 347;
  
  // VS Pump (from pump.rb)
  constexpr uint16_t VS_PUMP_MIN = 321;
  constexpr uint16_t VS_PUMP_MAX = 322;
  constexpr uint16_t VS_PUMP_MANUAL = 323;
  constexpr uint16_t VS_PUMP_SPEED = 325;
  
  // Refrigeration monitoring (from compressor.rb)
  constexpr uint16_t HEATING_LIQUID_LINE_TEMP = 1109;
  constexpr uint16_t SATURATED_CONDENSER_TEMP = 1134;
  constexpr uint16_t SUBCOOL_HEATING = 1135;
  constexpr uint16_t SUBCOOL_COOLING = 1136;
  
  // Energy monitoring (32-bit values, high word first)
  constexpr uint16_t COMPRESSOR_WATTS = 1146;
  constexpr uint16_t BLOWER_WATTS = 1148;
  constexpr uint16_t AUX_WATTS = 1150;
  constexpr uint16_t TOTAL_WATTS = 1152;
  constexpr uint16_t HEAT_OF_EXTRACTION = 1154;
  constexpr uint16_t HEAT_OF_REJECTION = 1156;
  constexpr uint16_t PUMP_WATTS = 1164;
  
  // VS Drive
  constexpr uint16_t COMPRESSOR_SPEED_DESIRED = 3000;
  constexpr uint16_t COMPRESSOR_SPEED_ACTUAL = 3001;
  constexpr uint16_t VS_DISCHARGE_PRESSURE = 3322;
  constexpr uint16_t VS_SUCTION_PRESSURE = 3323;
  constexpr uint16_t VS_DISCHARGE_TEMP = 3325;
  constexpr uint16_t VS_AMBIENT_TEMP = 3326;
  constexpr uint16_t VS_DRIVE_TEMP = 3327;
  constexpr uint16_t VS_COMPRESSOR_WATTS = 3422;   // + 3423 low word (uint32, Watts)
  constexpr uint16_t VS_INVERTER_TEMP = 3522;
  constexpr uint16_t VS_FAN_SPEED = 3524;           // Fan speed percentage (0-100%)
  constexpr uint16_t VS_EEV_OPEN = 3808;
  constexpr uint16_t VS_SUCTION_TEMP = 3903;
  constexpr uint16_t VS_SAT_EVAP_DISCHARGE_TEMP = 3905;  // Saturated evaporator discharge temp
  constexpr uint16_t VS_SUPERHEAT_TEMP = 3906;
  
  // Thermostat config (read)
  constexpr uint16_t FAN_CONFIG = 12005;
  constexpr uint16_t HEATING_MODE_READ = 12006;
  
  // Humidistat (from humidistat.rb)
  constexpr uint16_t HUMIDISTAT_SETTINGS = 12309;    // For non-IZ2
  constexpr uint16_t HUMIDISTAT_TARGETS = 12310;     // For non-IZ2
  constexpr uint16_t IZ2_HUMIDISTAT_SETTINGS = 21114;   // Write: mode bitmask (0x8000=auto humidify, 0x4000=auto dehumidify)
  constexpr uint16_t IZ2_HUMIDISTAT_TARGETS_WRITE = 21115;  // Write: (humidify_target << 8) | dehumidify_target
  constexpr uint16_t IZ2_HUMIDISTAT_MODE = 31109;       // Read: mode bitmask (same format as 21114)
  constexpr uint16_t IZ2_HUMIDISTAT_TARGETS = 31110;    // Read: (humidify_target << 8) | dehumidify_target
  
  // Thermostat config (write)
  constexpr uint16_t HEATING_MODE_WRITE = 12606;
  constexpr uint16_t HEATING_SETPOINT_WRITE = 12619;
  constexpr uint16_t COOLING_SETPOINT_WRITE = 12620;
  constexpr uint16_t FAN_MODE_WRITE = 12621;
  constexpr uint16_t FAN_INTERMITTENT_ON_WRITE = 12622;
  constexpr uint16_t FAN_INTERMITTENT_OFF_WRITE = 12623;
  
  // System commands
  constexpr uint16_t CLEAR_FAULT_HISTORY = 47;
  constexpr uint16_t CLEAR_FAULT_MAGIC = 0x5555;
  
  // VS Drive / active dehumidification
  constexpr uint16_t ACTIVE_DEHUMIDIFY = 362;
  
  // IZ2 system-wide desired speeds (from compressor.rb / blower.rb)
  constexpr uint16_t IZ2_COMPRESSOR_SPEED_DESIRED = 564;
  constexpr uint16_t IZ2_BLOWER_SPEED_DESIRED = 565;

  // IZ2 Zone registers (base addresses, add (zone-1)*offset for each zone)
  constexpr uint16_t IZ2_MODE_WRITE_BASE = 21202;      // +9 per zone
  constexpr uint16_t IZ2_HEAT_SP_WRITE_BASE = 21203;
  constexpr uint16_t IZ2_COOL_SP_WRITE_BASE = 21204;
  constexpr uint16_t IZ2_FAN_MODE_WRITE_BASE = 21205;
  constexpr uint16_t IZ2_FAN_ON_WRITE_BASE = 21206;
  constexpr uint16_t IZ2_FAN_OFF_WRITE_BASE = 21207;
  constexpr uint16_t IZ2_AMBIENT_BASE = 31007;         // +3 per zone
  constexpr uint16_t IZ2_CONFIG1_BASE = 31008;
  constexpr uint16_t IZ2_CONFIG2_BASE = 31009;
  constexpr uint16_t IZ2_CONFIG3_BASE = 31200;         // +3 per zone
  
  // Hardware detection registers
  constexpr uint16_t BLOWER_TYPE = 404;
  constexpr uint16_t ENERGY_MONITOR = 412;
  constexpr uint16_t PUMP_TYPE = 413;

  // IZ2 system registers
  constexpr uint16_t IZ2_INSTALLED = 812;
  constexpr uint16_t IZ2_VERSION = 813;
  constexpr uint16_t IZ2_NUM_ZONES = 483;
  constexpr uint16_t IZ2_OUTDOOR_TEMP = 31003;
  constexpr uint16_t IZ2_DEMAND = 31005;
}

// Flat sorted vector as register map — eliminates per-element heap overhead of std::map.
// With ~90 registers, this saves ~4KB of fragmented heap vs std::map's tree nodes.
// Lookup is O(log n) via std::lower_bound on a contiguous array (cache-friendly).
using RegisterMap = std::vector<std::pair<uint16_t, uint16_t>>;

// Find a register value in a sorted RegisterMap. Returns pointer to value or nullptr.
inline const uint16_t *reg_find(const RegisterMap &map, uint16_t addr) {
  auto it = std::lower_bound(map.begin(), map.end(), addr,
      [](const std::pair<uint16_t, uint16_t> &p, uint16_t a) { return p.first < a; });
  if (it != map.end() && it->first == addr)
    return &it->second;
  return nullptr;
}

// Insert or update a register value in a sorted RegisterMap.
inline void reg_insert(RegisterMap &map, uint16_t addr, uint16_t value) {
  auto it = std::lower_bound(map.begin(), map.end(), addr,
      [](const std::pair<uint16_t, uint16_t> &p, uint16_t a) { return p.first < a; });
  if (it != map.end() && it->first == addr) {
    it->second = value;
  } else {
    map.insert(it, {addr, value});
  }
}

// Blower type (register 404, from registers.rb BLOWER_TYPE)
enum class BlowerType : uint8_t {
  PSC = 0,
  ECM_208 = 1,
  ECM_265 = 2,
  FIVE_SPEED = 3
};

// Pump type (register 413, from registers.rb PUMP_TYPE)
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

// Maximum number of IZ2 zones
constexpr uint8_t MAX_IZ2_ZONES = 6;

// IZ2 Zone current mode/call (from registers.rb CALLS hash)
// Values must match the Ruby gem's CALLS mapping exactly:
//   0=standby, 1=unknown1, 2=h1, 3=h2, 4=h3, 5=c1, 6=c2, 7=unknown7
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

// Zone priority
enum class ZonePriority : uint8_t {
  COMFORT = 0,
  ECONOMY = 1
};

// Zone size
enum class ZoneSize : uint8_t {
  QUARTER = 0,
  HALF = 1,
  THREE_QUARTER = 2,
  FULL = 3
};

// Structure to hold IZ2 zone data
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

class WaterFurnaceAurora : public PollingComponent, public uart::UARTDevice {
 public:
  WaterFurnaceAurora() = default;

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_address(uint8_t address) { this->address_ = address; }
  void set_flow_control_pin(GPIOPin *pin) { this->flow_control_pin_ = pin; }
  void set_read_retries(uint8_t retries) { this->read_retries_ = retries; }

  // Hardware override setters (skip auto-detection when set)
  void set_has_axb_override(bool value) { this->has_axb_ = value; this->axb_override_ = true; }
  void set_has_vs_drive_override(bool value) { this->has_vs_drive_ = value; this->vs_drive_override_ = true; }
  void set_has_iz2_override(bool value) { this->has_iz2_ = value; this->iz2_override_ = true; }
  void set_num_iz2_zones_override(uint8_t value) { this->num_iz2_zones_ = value; this->iz2_zones_override_ = true; }

  // Register sensors
  void set_entering_air_sensor(sensor::Sensor *sensor) { entering_air_sensor_ = sensor; }
  void set_leaving_air_sensor(sensor::Sensor *sensor) { leaving_air_sensor_ = sensor; }
  void set_ambient_temp_sensor(sensor::Sensor *sensor) { ambient_temp_sensor_ = sensor; }
  void set_outdoor_temp_sensor(sensor::Sensor *sensor) { outdoor_temp_sensor_ = sensor; }
  void set_entering_water_sensor(sensor::Sensor *sensor) { entering_water_sensor_ = sensor; }
  void set_leaving_water_sensor(sensor::Sensor *sensor) { leaving_water_sensor_ = sensor; }
  void set_heating_setpoint_sensor(sensor::Sensor *sensor) { heating_setpoint_sensor_ = sensor; }
  void set_cooling_setpoint_sensor(sensor::Sensor *sensor) { cooling_setpoint_sensor_ = sensor; }
  void set_humidity_sensor(sensor::Sensor *sensor) { humidity_sensor_ = sensor; }
  void set_compressor_speed_sensor(sensor::Sensor *sensor) { compressor_speed_sensor_ = sensor; }
  void set_total_watts_sensor(sensor::Sensor *sensor) { total_watts_sensor_ = sensor; }
  void set_compressor_watts_sensor(sensor::Sensor *sensor) { compressor_watts_sensor_ = sensor; }
  void set_blower_watts_sensor(sensor::Sensor *sensor) { blower_watts_sensor_ = sensor; }
  void set_aux_watts_sensor(sensor::Sensor *sensor) { aux_watts_sensor_ = sensor; }
  void set_pump_watts_sensor(sensor::Sensor *sensor) { pump_watts_sensor_ = sensor; }
  void set_line_voltage_sensor(sensor::Sensor *sensor) { line_voltage_sensor_ = sensor; }
  void set_waterflow_sensor(sensor::Sensor *sensor) { waterflow_sensor_ = sensor; }
  void set_loop_pressure_sensor(sensor::Sensor *sensor) { loop_pressure_sensor_ = sensor; }
  void set_dhw_temp_sensor(sensor::Sensor *sensor) { dhw_temp_sensor_ = sensor; }
  void set_dhw_setpoint_sensor(sensor::Sensor *sensor) { dhw_setpoint_sensor_ = sensor; }
  void set_fault_code_sensor(sensor::Sensor *sensor) { fault_code_sensor_ = sensor; }
  void set_discharge_pressure_sensor(sensor::Sensor *sensor) { discharge_pressure_sensor_ = sensor; }
  void set_suction_pressure_sensor(sensor::Sensor *sensor) { suction_pressure_sensor_ = sensor; }
  void set_eev_open_sensor(sensor::Sensor *sensor) { eev_open_sensor_ = sensor; }
  void set_superheat_sensor(sensor::Sensor *sensor) { superheat_sensor_ = sensor; }
  void set_fp1_sensor(sensor::Sensor *sensor) { fp1_sensor_ = sensor; }
  void set_fp2_sensor(sensor::Sensor *sensor) { fp2_sensor_ = sensor; }
  void set_line_voltage_setting_sensor(sensor::Sensor *sensor) { line_voltage_setting_sensor_ = sensor; }
  void set_anti_short_cycle_sensor(sensor::Sensor *sensor) { anti_short_cycle_sensor_ = sensor; }
  
  // Additional VS Drive sensors
  void set_compressor_desired_speed_sensor(sensor::Sensor *sensor) { compressor_desired_speed_sensor_ = sensor; }
  void set_discharge_temp_sensor(sensor::Sensor *sensor) { discharge_temp_sensor_ = sensor; }
  void set_suction_temp_sensor(sensor::Sensor *sensor) { suction_temp_sensor_ = sensor; }
  void set_vs_drive_temp_sensor(sensor::Sensor *sensor) { vs_drive_temp_sensor_ = sensor; }
  void set_vs_inverter_temp_sensor(sensor::Sensor *sensor) { vs_inverter_temp_sensor_ = sensor; }
  
  // Additional VS Drive sensors (Phase 5 parity)
  void set_vs_fan_speed_sensor(sensor::Sensor *sensor) { vs_fan_speed_sensor_ = sensor; }
  void set_vs_ambient_temp_sensor(sensor::Sensor *sensor) { vs_ambient_temp_sensor_ = sensor; }
  void set_vs_compressor_watts_sensor(sensor::Sensor *sensor) { vs_compressor_watts_sensor_ = sensor; }
  void set_sat_evap_discharge_temp_sensor(sensor::Sensor *sensor) { sat_evap_discharge_temp_sensor_ = sensor; }
  void set_aux_heat_stage_sensor(sensor::Sensor *sensor) { aux_heat_stage_sensor_ = sensor; }

  // IZ2 desired speed sensors (from compressor.rb / blower.rb)
  void set_iz2_compressor_speed_sensor(sensor::Sensor *sensor) { iz2_compressor_speed_sensor_ = sensor; }
  void set_iz2_blower_speed_sensor(sensor::Sensor *sensor) { iz2_blower_speed_sensor_ = sensor; }

  // Derived sensors — computed on-device from raw register values.
  // These go beyond the Ruby gem which only publishes raw values.
  void set_cop_sensor(sensor::Sensor *sensor) { cop_sensor_ = sensor; }
  void set_water_delta_t_sensor(sensor::Sensor *sensor) { water_delta_t_sensor_ = sensor; }
  void set_approach_temp_sensor(sensor::Sensor *sensor) { approach_temp_sensor_ = sensor; }

  // Blower/ECM sensors
  void set_blower_speed_sensor(sensor::Sensor *sensor) { blower_speed_sensor_ = sensor; }
  void set_blower_only_speed_sensor(sensor::Sensor *sensor) { blower_only_speed_sensor_ = sensor; }
  void set_lo_compressor_speed_sensor(sensor::Sensor *sensor) { lo_compressor_speed_sensor_ = sensor; }
  void set_hi_compressor_speed_sensor(sensor::Sensor *sensor) { hi_compressor_speed_sensor_ = sensor; }
  void set_aux_heat_speed_sensor(sensor::Sensor *sensor) { aux_heat_speed_sensor_ = sensor; }
  
  // VS Pump sensors
  void set_pump_speed_sensor(sensor::Sensor *sensor) { pump_speed_sensor_ = sensor; }
  void set_pump_min_speed_sensor(sensor::Sensor *sensor) { pump_min_speed_sensor_ = sensor; }
  void set_pump_max_speed_sensor(sensor::Sensor *sensor) { pump_max_speed_sensor_ = sensor; }
  
  // Refrigeration monitoring sensors
  void set_heating_liquid_line_temp_sensor(sensor::Sensor *sensor) { heating_liquid_line_temp_sensor_ = sensor; }
  void set_saturated_condenser_temp_sensor(sensor::Sensor *sensor) { saturated_condenser_temp_sensor_ = sensor; }
  void set_subcool_temp_sensor(sensor::Sensor *sensor) { subcool_temp_sensor_ = sensor; }
  void set_heat_of_extraction_sensor(sensor::Sensor *sensor) { heat_of_extraction_sensor_ = sensor; }
  void set_heat_of_rejection_sensor(sensor::Sensor *sensor) { heat_of_rejection_sensor_ = sensor; }
  
  // Humidifier sensors
  void set_humidification_target_sensor(sensor::Sensor *sensor) { humidification_target_sensor_ = sensor; }
  void set_dehumidification_target_sensor(sensor::Sensor *sensor) { dehumidification_target_sensor_ = sensor; }
  
  // Binary sensors
  void set_compressor_binary_sensor(binary_sensor::BinarySensor *sensor) { compressor_sensor_ = sensor; }
  void set_blower_binary_sensor(binary_sensor::BinarySensor *sensor) { blower_sensor_ = sensor; }
  void set_aux_heat_binary_sensor(binary_sensor::BinarySensor *sensor) { aux_heat_sensor_ = sensor; }
  void set_dhw_running_binary_sensor(binary_sensor::BinarySensor *sensor) { dhw_running_sensor_ = sensor; }
  void set_lockout_binary_sensor(binary_sensor::BinarySensor *sensor) { lockout_sensor_ = sensor; }
  void set_loop_pump_binary_sensor(binary_sensor::BinarySensor *sensor) { loop_pump_sensor_ = sensor; }
  void set_humidifier_running_binary_sensor(binary_sensor::BinarySensor *sensor) { humidifier_running_sensor_ = sensor; }
  void set_dehumidifier_running_binary_sensor(binary_sensor::BinarySensor *sensor) { dehumidifier_running_sensor_ = sensor; }
  void set_lps_binary_sensor(binary_sensor::BinarySensor *sensor) { lps_sensor_ = sensor; }
  void set_hps_binary_sensor(binary_sensor::BinarySensor *sensor) { hps_sensor_ = sensor; }
  void set_emergency_shutdown_binary_sensor(binary_sensor::BinarySensor *sensor) { emergency_shutdown_sensor_ = sensor; }
  void set_load_shed_binary_sensor(binary_sensor::BinarySensor *sensor) { load_shed_sensor_ = sensor; }

  // Text sensors
  void set_current_mode_sensor(text_sensor::TextSensor *sensor) { current_mode_sensor_ = sensor; }
  void set_fault_description_sensor(text_sensor::TextSensor *sensor) { fault_description_sensor_ = sensor; }
  void set_hvac_mode_sensor(text_sensor::TextSensor *sensor) { hvac_mode_sensor_ = sensor; }
  void set_fan_mode_sensor(text_sensor::TextSensor *sensor) { fan_mode_sensor_ = sensor; }
  void set_model_number_sensor(text_sensor::TextSensor *sensor) { model_number_sensor_ = sensor; }
  void set_serial_number_sensor(text_sensor::TextSensor *sensor) { serial_number_sensor_ = sensor; }
  void set_fault_history_sensor(text_sensor::TextSensor *sensor) { fault_history_sensor_ = sensor; }
  void set_vs_derate_sensor(text_sensor::TextSensor *sensor) { vs_derate_sensor_ = sensor; }
  void set_vs_safe_mode_sensor(text_sensor::TextSensor *sensor) { vs_safe_mode_sensor_ = sensor; }
  void set_vs_alarm_sensor(text_sensor::TextSensor *sensor) { vs_alarm_sensor_ = sensor; }
  void set_axb_inputs_sensor(text_sensor::TextSensor *sensor) { axb_inputs_sensor_ = sensor; }
  void set_humidifier_mode_sensor(text_sensor::TextSensor *sensor) { humidifier_mode_sensor_ = sensor; }
  void set_dehumidifier_mode_sensor(text_sensor::TextSensor *sensor) { dehumidifier_mode_sensor_ = sensor; }
  void set_pump_type_sensor(text_sensor::TextSensor *sensor) { pump_type_sensor_ = sensor; }

  // Control methods (called by climate/water_heater components)
  bool set_heating_setpoint(float temp);
  bool set_cooling_setpoint(float temp);
  bool set_hvac_mode(HeatingMode mode);
  bool set_fan_mode(FanMode mode);
  bool set_dhw_enabled(bool enabled);
  bool set_dhw_setpoint(float temp);
  
  // Blower speed controls (from blower.rb)
  bool set_blower_only_speed(uint8_t speed);
  bool set_lo_compressor_speed(uint8_t speed);
  bool set_hi_compressor_speed(uint8_t speed);
  bool set_aux_heat_ecm_speed(uint8_t speed);
  
  // Pump speed controls (from pump.rb)
  bool set_pump_speed(uint8_t speed);
  bool set_pump_min_speed(uint8_t speed);
  bool set_pump_max_speed(uint8_t speed);
  
  // Fan intermittent timing (from thermostat.rb)
  bool set_fan_intermittent_on(uint8_t minutes);
  bool set_fan_intermittent_off(uint8_t minutes);
  
  // Humidifier controls (from humidistat.rb)
  bool set_humidification_target(uint8_t percent);
  bool set_dehumidification_target(uint8_t percent);
  
  // System controls
  bool clear_fault_history();
  
  // IZ2 Zone controls (zone_number is 1-6)
  bool set_zone_heating_setpoint(uint8_t zone_number, float temp);
  bool set_zone_cooling_setpoint(uint8_t zone_number, float temp);
  bool set_zone_hvac_mode(uint8_t zone_number, HeatingMode mode);
  bool set_zone_fan_mode(uint8_t zone_number, FanMode mode);
  bool set_zone_fan_intermittent_on(uint8_t zone_number, uint8_t minutes);
  bool set_zone_fan_intermittent_off(uint8_t zone_number, uint8_t minutes);

  // Getters for current state
  float get_ambient_temperature() const { return ambient_temp_; }
  float get_heating_setpoint() const { return heating_setpoint_; }
  float get_cooling_setpoint() const { return cooling_setpoint_; }
  HeatingMode get_hvac_mode() const { return hvac_mode_; }
  FanMode get_fan_mode() const { return fan_mode_; }
  bool is_dhw_enabled() const { return dhw_enabled_; }
  float get_dhw_setpoint() const { return dhw_setpoint_; }
  float get_dhw_temperature() const { return dhw_temp_; }
  uint16_t get_system_outputs() const { return system_outputs_; }
  uint16_t get_axb_outputs() const { return axb_outputs_; }
  bool is_locked_out() const { return locked_out_; }
  
  // Observer pattern: sub-entities register a callback to be notified when data updates.
  // Callbacks are invoked at the end of refresh_all_data(), replacing the 5-second
  // polling loop in each sub-entity. Matches the Ruby gem's "parent polls, pushes
  // to children" pattern (ABCClient#refresh → zone.refresh, component.refresh).
  void register_listener(std::function<void()> callback) {
    this->listeners_.push_back(std::move(callback));
  }

  // IZ2 Zone getters
  bool has_iz2() const { return has_iz2_; }
  uint8_t get_num_iz2_zones() const { return num_iz2_zones_; }
  const IZ2ZoneData& get_zone_data(uint8_t zone_number) const;

 protected:
  // WaterFurnace custom Modbus protocol implementation
  // Based on lib/aurora/modbus/slave.rb
  
  // Function code 0x41 ('A') - Read multiple register ranges
  // Request: [addr][0x41][start1_hi][start1_lo][count1_hi][count1_lo]...[CRC]
  // Response: [addr][0x41][byte_count][data...][CRC]
  bool read_register_ranges(const std::vector<std::pair<uint16_t, uint16_t>> &ranges,
                            RegisterMap &result);
  
  // Function code 0x42 ('B') - Read specific registers (non-contiguous)
  // Request: [addr][0x42][addr1_hi][addr1_lo][addr2_hi][addr2_lo]...[CRC]
  // Response: [addr][0x42][byte_count][data...][CRC]
  bool read_specific_registers(const std::vector<uint16_t> &addresses,
                               RegisterMap &result);
  
  // Standard Modbus function code 0x03 - Read holding registers
  bool read_holding_registers(uint16_t start_addr, uint16_t count, std::vector<uint16_t> &result);
  
  // Standard Modbus function code 0x06 - Write single register
  bool write_holding_register(uint16_t addr, uint16_t value);
  
  // CRC calculation (standard Modbus CRC-16)
  uint16_t calculate_crc(const uint8_t *data, size_t len);
  
  // Send a Modbus request and receive response (shared transport for all function codes)
  // Handles: clear pending data, TX/RX flow control, write, flush, delay, wait_for_response
  bool send_and_receive(const uint8_t *request, size_t request_len,
                        std::vector<uint8_t> &response, uint32_t timeout_ms = 1000);
  
  // Wait for and parse response
  bool wait_for_response(std::vector<uint8_t> &response, uint32_t timeout_ms = 1000);
  
  // Data parsing helpers (from registers.rb)
  static float to_signed_tenths(uint16_t value);
  static float to_tenths(uint16_t value);
  static uint32_t to_uint32(uint16_t high, uint16_t low);
  static int32_t to_int32(uint16_t high, uint16_t low);
  static std::string registers_to_string(const std::vector<uint16_t> &regs);
  
  // Refresh data from device
  void refresh_all_data();
  
  // Hardware auto-detection (called from setup)
  void detect_hardware();
  
  // Read model and serial number (called from setup)
  void read_device_info();
  
  // Get mode/fault strings (from registers.rb FAULTS hash)
  std::string get_current_mode_string();
  static const char* get_fault_description(uint8_t code);
  static const char* get_hvac_mode_string(HeatingMode mode);
  static const char* get_fan_mode_string(FanMode mode);
  static std::string get_vs_derate_string(uint16_t value);
  static std::string get_vs_safe_mode_string(uint16_t value);
  static std::string get_vs_alarm_string(uint16_t alarm1, uint16_t alarm2);
  static std::string get_axb_inputs_string(uint16_t value);
  
  // Zone number validation helper (DRY: used by all 6 zone control methods)
  bool validate_zone_number(uint8_t zone_number) const;
  
  // AWL version helpers (from Ruby gem abc_client.rb)
  bool awl_axb() const { return has_axb_ && axb_version_ >= 2.0f; }
  bool awl_thermostat() const { return thermostat_version_ >= 3.0f; }
  bool awl_iz2() const { return has_iz2_ && iz2_version_ >= 2.0f; }
  bool awl_communicating() const { return awl_thermostat() || awl_iz2(); }
  bool is_ecm_blower() const { return blower_type_ == BlowerType::ECM_208 || blower_type_ == BlowerType::ECM_265; }
  bool is_vs_pump() const { return pump_type_ == PumpType::VS_PUMP || pump_type_ == PumpType::VS_PUMP_26_99 || pump_type_ == PumpType::VS_PUMP_UPS26_99; }
  bool refrigeration_monitoring() const { return energy_monitor_level_ >= 1; }
  bool energy_monitoring() const { return energy_monitor_level_ >= 2; }
  static const char* get_pump_type_string(PumpType type);
  
  // IZ2 blower speed conversion (from registers.rb iz2_fan_desired)
  // Maps IZ2 speed codes 1-6 to percentages: 25%, 40%, 55%, 70%, 85%, 100%
  static uint8_t iz2_fan_desired(uint16_t value);

  // Sensor publication helpers — DRY extraction for the 50+ find-and-publish patterns.
  // Ruby gem equivalent: REGISTER_CONVERTERS hash mapping lambdas to register arrays.
  // Each helper finds the register in the result map, applies a conversion, and publishes.
  // Returns true if the register was found (regardless of whether a sensor was configured).
  
  // Publish raw uint16_t value
  bool publish_sensor(const RegisterMap &result, uint16_t reg,
                      sensor::Sensor *sensor) {
    const uint16_t *val = reg_find(result, reg);
    if (!val) return false;
    if (sensor != nullptr) sensor->publish_state(*val);
    return true;
  }
  
  // Publish with to_tenths() conversion (unsigned value / 10.0)
  bool publish_sensor_tenths(const RegisterMap &result, uint16_t reg,
                             sensor::Sensor *sensor) {
    const uint16_t *val = reg_find(result, reg);
    if (!val) return false;
    if (sensor != nullptr) sensor->publish_state(to_tenths(*val));
    return true;
  }
  
  // Publish with to_signed_tenths() conversion (signed value / 10.0)
  bool publish_sensor_signed_tenths(const RegisterMap &result, uint16_t reg,
                                     sensor::Sensor *sensor) {
    const uint16_t *val = reg_find(result, reg);
    if (!val) return false;
    if (sensor != nullptr) sensor->publish_state(to_signed_tenths(*val));
    return true;
  }
  
  // Publish 32-bit unsigned value from two consecutive registers (high, low)
  bool publish_sensor_uint32(const RegisterMap &result, uint16_t reg_high,
                              sensor::Sensor *sensor) {
    const uint16_t *val_h = reg_find(result, reg_high);
    const uint16_t *val_l = reg_find(result, reg_high + 1);
    if (!val_h || !val_l) return false;
    if (sensor != nullptr) sensor->publish_state(to_uint32(*val_h, *val_l));
    return true;
  }
  
  // Publish 32-bit signed value from two consecutive registers (high, low)
  bool publish_sensor_int32(const RegisterMap &result, uint16_t reg_high,
                             sensor::Sensor *sensor) {
    const uint16_t *val_h = reg_find(result, reg_high);
    const uint16_t *val_l = reg_find(result, reg_high + 1);
    if (!val_h || !val_l) return false;
    if (sensor != nullptr) sensor->publish_state(to_int32(*val_h, *val_l));
    return true;
  }

  // Publish text sensor only when value has changed (avoid redundant API traffic)
  void publish_text_if_changed(text_sensor::TextSensor *sensor, std::string &cached,
                                const std::string &value) {
    if (sensor != nullptr && value != cached) {
      sensor->publish_state(value);
      cached = value;
    }
  }
  void publish_text_if_changed(text_sensor::TextSensor *sensor, std::string &cached,
                                const char *value) {
    if (sensor != nullptr && cached != value) {
      sensor->publish_state(value);
      cached = value;
    }
  }
  
  // Compute and publish derived sensors (COP, delta-T, approach)
  void publish_derived_sensors(const RegisterMap &regs);
  
  // Read fault history (registers 601-699)
  void read_fault_history();

  uint8_t address_{1};
  uint8_t read_retries_{2};  // Number of retries on read failure (default 2, like Ruby)
  
  // RS485 flow control pin (optional, for half-duplex RS485)
  GPIOPin *flow_control_pin_{nullptr};
  
  // Hardware override flags (true = user explicitly set via YAML, skip auto-detection)
  bool axb_override_{false};
  bool vs_drive_override_{false};
  bool iz2_override_{false};
  bool iz2_zones_override_{false};
  
  // Cached register values — flat sorted vector, no per-element heap overhead
  RegisterMap register_cache_;
  
  // Pre-allocated vector for register addresses to reduce stack usage
  std::vector<uint16_t> addresses_to_read_;
  
  // State — NAN until first successful poll (matches Ruby gem's nil default)
  float ambient_temp_{NAN};
  float heating_setpoint_{NAN};
  float cooling_setpoint_{NAN};
  float dhw_temp_{NAN};
  float dhw_setpoint_{NAN};
  bool dhw_enabled_{false};
  HeatingMode hvac_mode_{HeatingMode::OFF};
  FanMode fan_mode_{FanMode::AUTO};
  uint16_t system_outputs_{0};
  uint16_t axb_outputs_{0};
  uint16_t current_fault_{0};
  bool locked_out_{false};
  bool has_axb_{false};
  bool has_vs_drive_{false};
  bool has_iz2_{false};
  uint8_t num_iz2_zones_{0};
  bool active_dehumidify_{false};

  // AWL version fields (from Ruby gem abc_client.rb)
  float thermostat_version_{0.0f};
  float axb_version_{0.0f};
  float iz2_version_{0.0f};

  // Hardware type detection (from Ruby gem abc_client.rb)
  BlowerType blower_type_{BlowerType::PSC};
  PumpType pump_type_{PumpType::OTHER};
  uint8_t energy_monitor_level_{0};  // 0=None, 1=Compressor Monitor, 2=Energy Monitor
  
  // IZ2 Zone data
  IZ2ZoneData iz2_zones_[MAX_IZ2_ZONES];
  
  // Sensors
  sensor::Sensor *entering_air_sensor_{nullptr};
  sensor::Sensor *leaving_air_sensor_{nullptr};
  sensor::Sensor *ambient_temp_sensor_{nullptr};
  sensor::Sensor *outdoor_temp_sensor_{nullptr};
  sensor::Sensor *entering_water_sensor_{nullptr};
  sensor::Sensor *leaving_water_sensor_{nullptr};
  sensor::Sensor *heating_setpoint_sensor_{nullptr};
  sensor::Sensor *cooling_setpoint_sensor_{nullptr};
  sensor::Sensor *humidity_sensor_{nullptr};
  sensor::Sensor *compressor_speed_sensor_{nullptr};
  sensor::Sensor *total_watts_sensor_{nullptr};
  sensor::Sensor *compressor_watts_sensor_{nullptr};
  sensor::Sensor *blower_watts_sensor_{nullptr};
  sensor::Sensor *aux_watts_sensor_{nullptr};
  sensor::Sensor *pump_watts_sensor_{nullptr};
  sensor::Sensor *line_voltage_sensor_{nullptr};
  sensor::Sensor *waterflow_sensor_{nullptr};
  sensor::Sensor *loop_pressure_sensor_{nullptr};
  sensor::Sensor *dhw_temp_sensor_{nullptr};
  sensor::Sensor *dhw_setpoint_sensor_{nullptr};
  sensor::Sensor *fault_code_sensor_{nullptr};
  sensor::Sensor *discharge_pressure_sensor_{nullptr};
  sensor::Sensor *suction_pressure_sensor_{nullptr};
  sensor::Sensor *eev_open_sensor_{nullptr};
  sensor::Sensor *superheat_sensor_{nullptr};
  sensor::Sensor *fp1_sensor_{nullptr};
  sensor::Sensor *fp2_sensor_{nullptr};
  sensor::Sensor *line_voltage_setting_sensor_{nullptr};
  sensor::Sensor *anti_short_cycle_sensor_{nullptr};
  
  // Additional VS Drive sensors
  sensor::Sensor *compressor_desired_speed_sensor_{nullptr};
  sensor::Sensor *discharge_temp_sensor_{nullptr};
  sensor::Sensor *suction_temp_sensor_{nullptr};
  sensor::Sensor *vs_drive_temp_sensor_{nullptr};
  sensor::Sensor *vs_inverter_temp_sensor_{nullptr};
  
  // Additional VS Drive sensors (Phase 5 parity)
  sensor::Sensor *vs_fan_speed_sensor_{nullptr};
  sensor::Sensor *vs_ambient_temp_sensor_{nullptr};
  sensor::Sensor *vs_compressor_watts_sensor_{nullptr};
  sensor::Sensor *sat_evap_discharge_temp_sensor_{nullptr};
  sensor::Sensor *aux_heat_stage_sensor_{nullptr};
  
  // IZ2 desired speed sensors
  sensor::Sensor *iz2_compressor_speed_sensor_{nullptr};
  sensor::Sensor *iz2_blower_speed_sensor_{nullptr};
  
  // Derived sensors (computed on-device, not in Ruby gem)
  sensor::Sensor *cop_sensor_{nullptr};
  sensor::Sensor *water_delta_t_sensor_{nullptr};
  sensor::Sensor *approach_temp_sensor_{nullptr};
  
  // Blower/ECM sensors
  sensor::Sensor *blower_speed_sensor_{nullptr};
  sensor::Sensor *blower_only_speed_sensor_{nullptr};
  sensor::Sensor *lo_compressor_speed_sensor_{nullptr};
  sensor::Sensor *hi_compressor_speed_sensor_{nullptr};
  sensor::Sensor *aux_heat_speed_sensor_{nullptr};
  
  // VS Pump sensors
  sensor::Sensor *pump_speed_sensor_{nullptr};
  sensor::Sensor *pump_min_speed_sensor_{nullptr};
  sensor::Sensor *pump_max_speed_sensor_{nullptr};
  
  // Refrigeration monitoring sensors
  sensor::Sensor *heating_liquid_line_temp_sensor_{nullptr};
  sensor::Sensor *saturated_condenser_temp_sensor_{nullptr};
  sensor::Sensor *subcool_temp_sensor_{nullptr};
  sensor::Sensor *heat_of_extraction_sensor_{nullptr};
  sensor::Sensor *heat_of_rejection_sensor_{nullptr};
  
  // Humidifier sensors
  sensor::Sensor *humidification_target_sensor_{nullptr};
  sensor::Sensor *dehumidification_target_sensor_{nullptr};
  
  // Binary sensors
  binary_sensor::BinarySensor *compressor_sensor_{nullptr};
  binary_sensor::BinarySensor *blower_sensor_{nullptr};
  binary_sensor::BinarySensor *aux_heat_sensor_{nullptr};
  binary_sensor::BinarySensor *dhw_running_sensor_{nullptr};
  binary_sensor::BinarySensor *lockout_sensor_{nullptr};
  binary_sensor::BinarySensor *loop_pump_sensor_{nullptr};
  binary_sensor::BinarySensor *humidifier_running_sensor_{nullptr};
  binary_sensor::BinarySensor *dehumidifier_running_sensor_{nullptr};
  binary_sensor::BinarySensor *lps_sensor_{nullptr};
  binary_sensor::BinarySensor *hps_sensor_{nullptr};
  binary_sensor::BinarySensor *emergency_shutdown_sensor_{nullptr};
  binary_sensor::BinarySensor *load_shed_sensor_{nullptr};

  // Text sensors
  text_sensor::TextSensor *current_mode_sensor_{nullptr};
  text_sensor::TextSensor *fault_description_sensor_{nullptr};
  text_sensor::TextSensor *hvac_mode_sensor_{nullptr};
  text_sensor::TextSensor *fan_mode_sensor_{nullptr};
  text_sensor::TextSensor *model_number_sensor_{nullptr};
  text_sensor::TextSensor *serial_number_sensor_{nullptr};
  text_sensor::TextSensor *fault_history_sensor_{nullptr};
  text_sensor::TextSensor *vs_derate_sensor_{nullptr};
  text_sensor::TextSensor *vs_safe_mode_sensor_{nullptr};
  text_sensor::TextSensor *vs_alarm_sensor_{nullptr};
  text_sensor::TextSensor *axb_inputs_sensor_{nullptr};
  text_sensor::TextSensor *humidifier_mode_sensor_{nullptr};
  text_sensor::TextSensor *dehumidifier_mode_sensor_{nullptr};
  text_sensor::TextSensor *pump_type_sensor_{nullptr};
  
  // Observer callbacks — sub-entities register to be notified on data update
  std::vector<std::function<void()>> listeners_;
  
  // Cached text sensor values for publish-on-change (avoid republishing identical strings)
  std::string cached_mode_string_;
  std::string cached_fault_description_;
  std::string cached_hvac_mode_;
  std::string cached_fan_mode_;
  std::string cached_vs_derate_;
  std::string cached_vs_safe_mode_;
  std::string cached_vs_alarm_;
  std::string cached_axb_inputs_;
  std::string cached_humidifier_mode_;
  std::string cached_dehumidifier_mode_;
  
  // Adaptive polling tier counter — medium/slow registers update less frequently
  uint8_t poll_tier_counter_{0};
  
  // Cached device info
  std::string model_number_;
  std::string serial_number_;
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
