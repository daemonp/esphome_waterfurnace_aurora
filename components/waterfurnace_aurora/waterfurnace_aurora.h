#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#include <vector>
#include <map>

namespace esphome {
namespace waterfurnace_aurora {

// WaterFurnace Aurora uses custom Modbus function codes:
// Function Code 0x41 ('A') - Read multiple register ranges (address + length pairs)
// Function Code 0x42 ('B') - Read specific individual registers (list of addresses)
// Function Code 0x03 - Standard read holding registers (also supported)
// Function Code 0x06 - Standard write single register

// Heating modes (from registers.rb HEATING_MODE)
enum HeatingMode : uint8_t {
  HEATING_MODE_OFF = 0,
  HEATING_MODE_AUTO = 1,
  HEATING_MODE_COOL = 2,
  HEATING_MODE_HEAT = 3,
  HEATING_MODE_EHEAT = 4
};

// Fan modes (from registers.rb FAN_MODE)
enum FanMode : uint8_t {
  FAN_MODE_AUTO = 0,
  FAN_MODE_CONTINUOUS = 1,
  FAN_MODE_INTERMITTENT = 2
};

// System outputs bitmask (register 30, from registers.rb SYSTEM_OUTPUTS)
static const uint16_t OUTPUT_CC = 0x01;       // Compressor stage 1
static const uint16_t OUTPUT_CC2 = 0x02;      // Compressor stage 2
static const uint16_t OUTPUT_RV = 0x04;       // Reversing valve (cool instead of heat)
static const uint16_t OUTPUT_BLOWER = 0x08;
static const uint16_t OUTPUT_EH1 = 0x10;      // Aux heat 1
static const uint16_t OUTPUT_EH2 = 0x20;      // Aux heat 2
static const uint16_t OUTPUT_ACCESSORY = 0x200;
static const uint16_t OUTPUT_LOCKOUT = 0x400;
static const uint16_t OUTPUT_ALARM = 0x800;

// AXB outputs bitmask (register 1104, from registers.rb AXB_OUTPUTS)
static const uint16_t AXB_OUTPUT_DHW = 0x01;
static const uint16_t AXB_OUTPUT_LOOP_PUMP = 0x02;
static const uint16_t AXB_OUTPUT_DIVERTING_VALVE = 0x04;

// Key register addresses (from registers.rb)
namespace registers {
  // System info
  static const uint16_t ABC_VERSION = 2;
  static const uint16_t COMPRESSOR_ANTI_SHORT_CYCLE = 6;
  static const uint16_t LINE_VOLTAGE = 16;
  static const uint16_t FP1_TEMP = 19;           // Cooling liquid line temp
  static const uint16_t FP2_TEMP = 20;           // Air coil temp
  static const uint16_t LAST_FAULT = 25;
  static const uint16_t SYSTEM_OUTPUTS = 30;
  static const uint16_t SYSTEM_STATUS = 31;
  static const uint16_t LINE_VOLTAGE_SETTING = 112;
  
  // DHW (requires AXB)
  static const uint16_t DHW_ENABLED = 400;
  static const uint16_t DHW_SETPOINT = 401;
  
  // Thermostat
  static const uint16_t AMBIENT_TEMP = 502;
  static const uint16_t ENTERING_AIR = 567;
  static const uint16_t ENTERING_AIR_AWL = 740;
  static const uint16_t RELATIVE_HUMIDITY = 741;
  static const uint16_t OUTDOOR_TEMP = 742;
  static const uint16_t HEATING_SETPOINT = 745;
  static const uint16_t COOLING_SETPOINT = 746;
  static const uint16_t AXB_INSTALLED = 806;
  static const uint16_t LEAVING_AIR = 900;
  
  // AXB
  static const uint16_t AXB_INPUTS = 1103;
  static const uint16_t AXB_OUTPUTS = 1104;
  static const uint16_t LEAVING_WATER = 1110;
  static const uint16_t ENTERING_WATER = 1111;
  static const uint16_t DHW_TEMP = 1114;
  static const uint16_t DISCHARGE_PRESSURE = 1115;
  static const uint16_t SUCTION_PRESSURE = 1116;
  static const uint16_t WATERFLOW = 1117;
  static const uint16_t LOOP_PRESSURE = 1119;
  
  // Energy monitoring (32-bit values, high word first)
  static const uint16_t COMPRESSOR_WATTS = 1146;
  static const uint16_t BLOWER_WATTS = 1148;
  static const uint16_t AUX_WATTS = 1150;
  static const uint16_t TOTAL_WATTS = 1152;
  static const uint16_t PUMP_WATTS = 1164;
  
  // VS Drive
  static const uint16_t COMPRESSOR_SPEED_DESIRED = 3000;
  static const uint16_t COMPRESSOR_SPEED_ACTUAL = 3001;
  static const uint16_t VS_DISCHARGE_PRESSURE = 3322;
  static const uint16_t VS_SUCTION_PRESSURE = 3323;
  static const uint16_t VS_DISCHARGE_TEMP = 3325;
  static const uint16_t VS_AMBIENT_TEMP = 3326;
  static const uint16_t VS_DRIVE_TEMP = 3327;
  static const uint16_t VS_INVERTER_TEMP = 3522;
  static const uint16_t VS_EEV_OPEN = 3808;
  static const uint16_t VS_SUCTION_TEMP = 3903;
  static const uint16_t VS_SUPERHEAT_TEMP = 3906;
  
  // Thermostat config (read)
  static const uint16_t FAN_CONFIG = 12005;
  static const uint16_t HEATING_MODE_READ = 12006;
  
  // Thermostat config (write)
  static const uint16_t HEATING_MODE_WRITE = 12606;
  static const uint16_t HEATING_SETPOINT_WRITE = 12619;
  static const uint16_t COOLING_SETPOINT_WRITE = 12620;
  static const uint16_t FAN_MODE_WRITE = 12621;
  
  // IZ2 Zone registers (base addresses, add (zone-1)*offset for each zone)
  static const uint16_t IZ2_MODE_WRITE_BASE = 21202;      // +9 per zone
  static const uint16_t IZ2_HEAT_SP_WRITE_BASE = 21203;
  static const uint16_t IZ2_COOL_SP_WRITE_BASE = 21204;
  static const uint16_t IZ2_FAN_MODE_WRITE_BASE = 21205;
  static const uint16_t IZ2_AMBIENT_BASE = 31007;         // +3 per zone
  static const uint16_t IZ2_CONFIG1_BASE = 31008;
  static const uint16_t IZ2_CONFIG2_BASE = 31009;
}

class WaterFurnaceAurora : public PollingComponent, public uart::UARTDevice {
 public:
  WaterFurnaceAurora() = default;

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_address(uint8_t address) { this->address_ = address; }

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
  
  // Binary sensors
  void set_compressor_binary_sensor(binary_sensor::BinarySensor *sensor) { compressor_sensor_ = sensor; }
  void set_blower_binary_sensor(binary_sensor::BinarySensor *sensor) { blower_sensor_ = sensor; }
  void set_aux_heat_binary_sensor(binary_sensor::BinarySensor *sensor) { aux_heat_sensor_ = sensor; }
  void set_dhw_running_binary_sensor(binary_sensor::BinarySensor *sensor) { dhw_running_sensor_ = sensor; }
  void set_lockout_binary_sensor(binary_sensor::BinarySensor *sensor) { lockout_sensor_ = sensor; }
  void set_loop_pump_binary_sensor(binary_sensor::BinarySensor *sensor) { loop_pump_sensor_ = sensor; }

  // Text sensors
  void set_current_mode_sensor(text_sensor::TextSensor *sensor) { current_mode_sensor_ = sensor; }
  void set_fault_description_sensor(text_sensor::TextSensor *sensor) { fault_description_sensor_ = sensor; }
  void set_hvac_mode_sensor(text_sensor::TextSensor *sensor) { hvac_mode_sensor_ = sensor; }
  void set_fan_mode_sensor(text_sensor::TextSensor *sensor) { fan_mode_sensor_ = sensor; }

  // Control methods (called by climate/water_heater components)
  bool set_heating_setpoint(float temp);
  bool set_cooling_setpoint(float temp);
  bool set_hvac_mode(HeatingMode mode);
  bool set_fan_mode(FanMode mode);
  bool set_dhw_enabled(bool enabled);
  bool set_dhw_setpoint(float temp);

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

 protected:
  // WaterFurnace custom Modbus protocol implementation
  // Based on lib/aurora/modbus/slave.rb
  
  // Function code 0x41 ('A') - Read multiple register ranges
  // Request: [addr][0x41][start1_hi][start1_lo][count1_hi][count1_lo]...[CRC]
  // Response: [addr][0x41][byte_count][data...][CRC]
  bool read_register_ranges(const std::vector<std::pair<uint16_t, uint16_t>> &ranges,
                            std::map<uint16_t, uint16_t> &result);
  
  // Function code 0x42 ('B') - Read specific registers (non-contiguous)
  // Request: [addr][0x42][addr1_hi][addr1_lo][addr2_hi][addr2_lo]...[CRC]
  // Response: [addr][0x42][byte_count][data...][CRC]
  bool read_specific_registers(const std::vector<uint16_t> &addresses,
                               std::map<uint16_t, uint16_t> &result);
  
  // Standard Modbus function code 0x03 - Read holding registers
  bool read_holding_registers(uint16_t start_addr, uint16_t count, std::vector<uint16_t> &result);
  
  // Standard Modbus function code 0x06 - Write single register
  bool write_holding_register(uint16_t addr, uint16_t value);
  
  // CRC calculation (standard Modbus CRC-16)
  uint16_t calculate_crc(const uint8_t *data, size_t len);
  
  // Wait for and parse response
  bool wait_for_response(std::vector<uint8_t> &response, uint32_t timeout_ms = 1000);
  
  // Data parsing helpers (from registers.rb)
  static float to_signed_tenths(uint16_t value);
  static float to_tenths(uint16_t value);
  static uint32_t to_uint32(uint16_t high, uint16_t low);
  static int32_t to_int32(uint16_t high, uint16_t low);
  
  // Refresh data from device
  void refresh_all_data();
  
  // Get mode/fault strings (from registers.rb FAULTS hash)
  std::string get_current_mode_string();
  static const char* get_fault_description(uint8_t code);
  static const char* get_hvac_mode_string(HeatingMode mode);
  static const char* get_fan_mode_string(FanMode mode);

  uint8_t address_{1};
  
  // Cached register values
  std::map<uint16_t, uint16_t> register_cache_;
  
  // State
  float ambient_temp_{NAN};
  float heating_setpoint_{NAN};
  float cooling_setpoint_{NAN};
  float dhw_temp_{NAN};
  float dhw_setpoint_{NAN};
  bool dhw_enabled_{false};
  HeatingMode hvac_mode_{HEATING_MODE_OFF};
  FanMode fan_mode_{FAN_MODE_AUTO};
  uint16_t system_outputs_{0};
  uint16_t axb_outputs_{0};
  uint16_t current_fault_{0};
  bool locked_out_{false};
  bool has_axb_{false};
  bool has_vs_drive_{false};
  bool active_dehumidify_{false};
  
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
  
  // Binary sensors
  binary_sensor::BinarySensor *compressor_sensor_{nullptr};
  binary_sensor::BinarySensor *blower_sensor_{nullptr};
  binary_sensor::BinarySensor *aux_heat_sensor_{nullptr};
  binary_sensor::BinarySensor *dhw_running_sensor_{nullptr};
  binary_sensor::BinarySensor *lockout_sensor_{nullptr};
  binary_sensor::BinarySensor *loop_pump_sensor_{nullptr};

  // Text sensors
  text_sensor::TextSensor *current_mode_sensor_{nullptr};
  text_sensor::TextSensor *fault_description_sensor_{nullptr};
  text_sensor::TextSensor *hvac_mode_sensor_{nullptr};
  text_sensor::TextSensor *fan_mode_sensor_{nullptr};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
