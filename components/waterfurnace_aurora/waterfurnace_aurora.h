#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

#ifdef USE_API_CUSTOM_SERVICES
#include "esphome/components/api/custom_api_device.h"
#endif

#include "registers.h"
#include "protocol.h"

#include <cmath>
#include <cstring>
#include <functional>
#include <utility>
#include <vector>

namespace esphome {
namespace waterfurnace_aurora {

// ============================================================================
// State Machine
// ============================================================================

enum class State : uint8_t {
  SETUP_READ_ID,            // Read model/serial/program info
  SETUP_DETECT_COMPONENTS,  // Detect AXB, VS Drive, IZ2, blower, pump, energy
  SETUP_DETECT_VS,          // VS Drive probe (optional second detect step)
  IDLE,                     // Ready for next operation
  TX_PENDING,               // Frame written to UART TX FIFO, waiting for transmit to complete
  WAITING_RESPONSE,         // Request sent, collecting bytes
  ERROR_BACKOFF,            // Communication error, waiting before retry
};

// Identifies what type of request is currently in-flight (for response routing)
enum class PendingRequest : uint8_t {
  NONE,
  SETUP_ID,          // func 0x03 read for model/serial
  SETUP_DETECT,      // func 0x42 read for hardware detection
  SETUP_VS_PROBE,    // func 0x42 read for VS drive probing
  POLL_REGISTERS,    // func 0x42 read for normal polling
  POLL_FAULT_HISTORY,// func 0x03 read for fault history
  WRITE_SINGLE,      // func 0x06 write
  // Note: func 0x43 (batch write) is supported by the protocol layer but NOT used
  // by the hub. The Aurora firmware rejects 0x43 with error 0x02. All writes use
  // individual 0x06 calls, matching the ccutrer/waterfurnace_aurora Ruby gem behavior.
};

// ============================================================================
// Hub Class
// ============================================================================

class WaterFurnaceAurora : public PollingComponent, public uart::UARTDevice
#ifdef USE_API_CUSTOM_SERVICES
    , public api::CustomAPIDevice
#endif
{
 public:
  // --- Timing constants ---
  static constexpr uint32_t RESPONSE_TIMEOUT_MS = 2000;
  static constexpr uint32_t ERROR_BACKOFF_MS = 5000;
  static constexpr size_t MAX_LISTENERS = 16;
  static constexpr uint32_t DEFAULT_CONNECTED_TIMEOUT_MS = 30000;
  static constexpr uint32_t WRITE_COOLDOWN_MS = 7000;

  WaterFurnaceAurora() = default;

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  void on_shutdown() override;
  
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_address(uint8_t address) { this->address_ = address; }
  void set_flow_control_pin(GPIOPin *pin) { this->flow_control_pin_ = pin; }
  void set_read_retries(uint8_t retries) { this->read_retries_ = retries; }

  // Hardware override setters (skip auto-detection when set)
  void set_has_axb_override(bool value) { this->has_axb_ = value; this->axb_override_ = true; }
  void set_has_vs_drive_override(bool value) { this->has_vs_drive_ = value; this->vs_drive_override_ = true; }
  void set_has_iz2_override(bool value) { this->has_iz2_ = value; this->iz2_override_ = true; }
  void set_num_iz2_zones_override(uint8_t value) { this->num_iz2_zones_ = value; this->iz2_zones_override_ = true; }

  // Connected sensor
  void set_connected_sensor(binary_sensor::BinarySensor *sensor) { this->connected_sensor_ = sensor; }
  void set_connected_timeout(uint32_t ms) { this->connected_timeout_ = ms; }

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
  
  // VS Drive additional diagnostics
  void set_vs_entering_water_temp_sensor(sensor::Sensor *sensor) { vs_entering_water_temp_sensor_ = sensor; }
  void set_vs_line_voltage_sensor(sensor::Sensor *sensor) { vs_line_voltage_sensor_ = sensor; }
  void set_vs_thermo_power_sensor(sensor::Sensor *sensor) { vs_thermo_power_sensor_ = sensor; }
  void set_vs_supply_voltage_sensor(sensor::Sensor *sensor) { vs_supply_voltage_sensor_ = sensor; }
  void set_vs_udc_voltage_sensor(sensor::Sensor *sensor) { vs_udc_voltage_sensor_ = sensor; }
  
  // AXB current sensors
  void set_blower_amps_sensor(sensor::Sensor *sensor) { blower_amps_sensor_ = sensor; }
  void set_aux_amps_sensor(sensor::Sensor *sensor) { aux_amps_sensor_ = sensor; }
  void set_compressor1_amps_sensor(sensor::Sensor *sensor) { compressor1_amps_sensor_ = sensor; }
  void set_compressor2_amps_sensor(sensor::Sensor *sensor) { compressor2_amps_sensor_ = sensor; }

  // IZ2 desired speed sensors
  void set_iz2_compressor_speed_sensor(sensor::Sensor *sensor) { iz2_compressor_speed_sensor_ = sensor; }
  void set_iz2_blower_speed_sensor(sensor::Sensor *sensor) { iz2_blower_speed_sensor_ = sensor; }

  // Derived sensors
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
  void set_fan_call_binary_sensor(binary_sensor::BinarySensor *sensor) { fan_call_sensor_ = sensor; }
  void set_derated_binary_sensor(binary_sensor::BinarySensor *sensor) { derated_sensor_ = sensor; }
  void set_safe_mode_binary_sensor(binary_sensor::BinarySensor *sensor) { safe_mode_sensor_ = sensor; }
  void set_diverting_valve_binary_sensor(binary_sensor::BinarySensor *sensor) { diverting_valve_sensor_ = sensor; }

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
  void set_lockout_fault_sensor(sensor::Sensor *sensor) { lockout_fault_sensor_ = sensor; }
  void set_lockout_fault_description_sensor(text_sensor::TextSensor *sensor) { lockout_fault_description_sensor_ = sensor; }
  void set_outputs_at_lockout_sensor(text_sensor::TextSensor *sensor) { outputs_at_lockout_sensor_ = sensor; }
  void set_inputs_at_lockout_sensor(text_sensor::TextSensor *sensor) { inputs_at_lockout_sensor_ = sensor; }

  // Control methods — now queue writes instead of blocking
  void write_register(uint16_t addr, uint16_t value);
  
  bool set_heating_setpoint(float temp);
  bool set_cooling_setpoint(float temp);
  bool set_hvac_mode(HeatingMode mode);
  bool set_fan_mode(FanMode mode);
  bool set_dhw_enabled(bool enabled);
  bool set_dhw_setpoint(float temp);
  
  // Blower speed controls
  bool set_blower_only_speed(uint8_t speed);
  bool set_lo_compressor_speed(uint8_t speed);
  bool set_hi_compressor_speed(uint8_t speed);
  bool set_aux_heat_ecm_speed(uint8_t speed);
  
  // Pump speed controls
  bool set_pump_speed(uint8_t speed);
  bool set_pump_min_speed(uint8_t speed);
  bool set_pump_max_speed(uint8_t speed);
  
  // Fan intermittent timing
  bool set_fan_intermittent_on(uint8_t minutes);
  bool set_fan_intermittent_off(uint8_t minutes);
  
  // Humidifier controls
  bool set_humidification_target(uint8_t percent);
  bool set_dehumidification_target(uint8_t percent);
  bool set_humidifier_mode(bool auto_mode);
  bool set_dehumidifier_mode(bool auto_mode);
  
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
  
  // Cooldown state queries — used by climate entities to skip stale overwrites
  bool setpoint_cooldown_active() const { return (millis() - last_setpoint_write_) <= WRITE_COOLDOWN_MS; }
  bool mode_cooldown_active() const { return (millis() - last_mode_write_) <= WRITE_COOLDOWN_MS; }
  bool fan_cooldown_active() const { return (millis() - last_fan_write_) <= WRITE_COOLDOWN_MS; }
  bool dhw_cooldown_active() const { return (millis() - last_dhw_write_) <= WRITE_COOLDOWN_MS; }
  bool humidity_target_cooldown_active() const { return (millis() - last_humidity_target_write_) <= WRITE_COOLDOWN_MS; }
  bool is_dhw_enabled() const { return dhw_enabled_; }
  float get_dhw_setpoint() const { return dhw_setpoint_; }
  float get_dhw_temperature() const { return dhw_temp_; }
  uint16_t get_system_outputs() const { return system_outputs_; }
  uint16_t get_axb_outputs() const { return axb_outputs_; }
  bool is_locked_out() const { return locked_out_; }
  bool is_setup_complete() const { return setup_complete_; }
  bool is_active_dehumidify() const { return active_dehumidify_; }
  float get_relative_humidity() const { return relative_humidity_; }
  bool get_humidifier_auto() const { return humidifier_auto_; }
  bool get_dehumidifier_auto() const { return dehumidifier_auto_; }
  bool awl_communicating() const { return awl_thermostat() || awl_iz2(); }
  
  /// Look up a raw register value from the cache. Returns NAN if not found.
  /// Used by AuroraNumber entities to get current read-back values.
  float get_cached_register(uint16_t addr) const {
    const uint16_t *val = reg_find(this->register_cache_, addr);
    return val ? static_cast<float>(*val) : NAN;
  }
  
  /// Look up a register value and return it divided by 10. Returns NAN if not found.
  float get_cached_register_tenths(uint16_t addr) const {
    const uint16_t *val = reg_find(this->register_cache_, addr);
    return val ? to_tenths(*val) : NAN;
  }
  
  // Observer pattern: sub-entities register a callback to be notified when data updates.
  void register_listener(std::function<void()> callback) {
    if (this->listeners_.size() >= MAX_LISTENERS) {
      // Bounded by YAML config; if we hit this, the YAML has more sub-entities than expected.
      return;
    }
    this->listeners_.push_back(std::move(callback));
  }

  // Deferred setup callbacks — fired when hardware detection completes.
  // If setup is already complete, the callback fires immediately.
  void register_setup_callback(std::function<void()> callback) {
    if (this->setup_complete_) {
      callback();
    } else {
      this->setup_callbacks_.push_back(std::move(callback));
    }
  }

  // IZ2 Zone getters
  bool has_iz2() const { return has_iz2_; }
  uint8_t get_num_iz2_zones() const { return num_iz2_zones_; }
  const IZ2ZoneData& get_zone_data(uint8_t zone_number) const;

 protected:
  // --- State machine operations ---
  void transition_(State new_state);
  /// Send a Modbus request frame and transition to WAITING_RESPONSE.
  /// expected_addrs is copied into the member; use the move overload when possible.
  void send_request_(const std::vector<uint8_t> &frame, PendingRequest type,
                     const std::vector<uint16_t> &expected_addrs);
  void send_request_(const std::vector<uint8_t> &frame, PendingRequest type,
                     std::vector<uint16_t> &&expected_addrs);
  /// Overload with no expected addresses (for writes).
  void send_request_(const std::vector<uint8_t> &frame, PendingRequest type);
  bool read_frame_(std::vector<uint8_t> &frame);
  void process_response_(const std::vector<uint8_t> &frame);
  void handle_timeout_();
  
  // --- Setup steps (called from loop() state machine) ---
  void start_setup_read_id_();
  void start_setup_detect_();
  void start_setup_vs_probe_();
  void process_setup_id_response_(const protocol::ParsedResponse &resp);
  void process_setup_detect_response_(const protocol::ParsedResponse &resp);
  void process_setup_vs_probe_response_(const protocol::ParsedResponse &resp);
  void finish_setup_();
  
  // --- Polling ---
  void start_poll_cycle_();
  void start_fault_history_read_();
  void process_poll_response_(const protocol::ParsedResponse &resp);
  void process_fault_history_response_(const protocol::ParsedResponse &resp);
  void publish_all_sensors_();
  
  // --- Write handling ---
  void process_pending_writes_();
  
  // --- Connectivity ---
  void update_connected_(bool connected);
  
  // Current mode string (computed from system outputs and state).
  // Returns a string literal (const char*) to avoid heap allocation in loop().
  const char *get_current_mode_string();
  
  // Zone number validation helper
  bool validate_zone_number(uint8_t zone_number) const;

#ifdef USE_API_CUSTOM_SERVICES
  // HA API custom service handler
  void on_write_register_service_(int32_t address, int32_t value);
#endif
  
  /// Estimate TX time in milliseconds for a given frame size at 19200 baud.
  /// 19200 baud with 8E1 = 11 bits/byte → ~0.573ms/byte. We add 1ms margin.
  uint32_t tx_time_ms_(size_t frame_bytes) const {
    return static_cast<uint32_t>((frame_bytes * 11 * 1000) / 19200) + 2;
  }

  // AWL version helpers
  bool awl_axb() const { return has_axb_ && axb_version_ >= 2.0f; }
  bool awl_thermostat() const { return thermostat_version_ >= 3.0f; }
  bool awl_iz2() const { return has_iz2_ && iz2_version_ >= 2.0f; }
  bool is_ecm_blower() const { return blower_type_ == BlowerType::ECM_208 || blower_type_ == BlowerType::ECM_265; }
  bool is_vs_pump() const { return pump_type_ == PumpType::VS_PUMP || pump_type_ == PumpType::VS_PUMP_26_99 || pump_type_ == PumpType::VS_PUMP_UPS26_99; }
  bool refrigeration_monitoring() const { return energy_monitor_level_ >= 1; }
  bool energy_monitoring() const { return energy_monitor_level_ >= 2; }

  // Build the addresses list for the current poll cycle based on tier
  void build_poll_addresses_();

  // Sensor publication helpers — DRY extraction for the 50+ find-and-publish patterns.
  // Each helper skips publish_state() if the sensor already has state and the value
  // is unchanged. This avoids ~5-7ms of API/TCP overhead per sensor per cycle,
  // reducing total publish time from ~333ms to ~20-40ms for typical polling.
  static bool sensor_value_changed_(sensor::Sensor *sensor, float value) {
    if (sensor == nullptr) return false;
    if (!sensor->has_state()) return true;  // First publish — always send
    // Both NaN → no change; one NaN → changed; otherwise compare values
    if (std::isnan(value)) return !std::isnan(sensor->raw_state);
    if (std::isnan(sensor->raw_state)) return true;
    return value != sensor->raw_state;
  }

  bool publish_sensor(const RegisterMap &result, uint16_t reg,
                      sensor::Sensor *sensor) {
    const uint16_t *val = reg_find(result, reg);
    if (!val) return false;
    float fval = static_cast<float>(*val);
    if (sensor_value_changed_(sensor, fval)) sensor->publish_state(fval);
    return true;
  }
  
  bool publish_sensor_tenths(const RegisterMap &result, uint16_t reg,
                             sensor::Sensor *sensor) {
    const uint16_t *val = reg_find(result, reg);
    if (!val) return false;
    float fval = to_tenths(*val);
    if (sensor_value_changed_(sensor, fval)) sensor->publish_state(fval);
    return true;
  }
  
  bool publish_sensor_signed_tenths(const RegisterMap &result, uint16_t reg,
                                     sensor::Sensor *sensor) {
    const uint16_t *val = reg_find(result, reg);
    if (!val) return false;
    float fval = to_signed_tenths(*val);
    if (sensor_value_changed_(sensor, fval)) sensor->publish_state(fval);
    return true;
  }
  
  bool publish_sensor_uint32(const RegisterMap &result, uint16_t reg_high,
                              sensor::Sensor *sensor) {
    const uint16_t *val_h = reg_find(result, reg_high);
    const uint16_t *val_l = reg_find(result, reg_high + 1);
    if (!val_h || !val_l) return false;
    float fval = static_cast<float>(to_uint32(*val_h, *val_l));
    if (sensor_value_changed_(sensor, fval)) sensor->publish_state(fval);
    return true;
  }
  
  bool publish_sensor_int32(const RegisterMap &result, uint16_t reg_high,
                             sensor::Sensor *sensor) {
    const uint16_t *val_h = reg_find(result, reg_high);
    const uint16_t *val_l = reg_find(result, reg_high + 1);
    if (!val_h || !val_l) return false;
    float fval = static_cast<float>(to_int32(*val_h, *val_l));
    if (sensor_value_changed_(sensor, fval)) sensor->publish_state(fval);
    return true;
  }

  /// Publish a binary sensor only if its value has changed from the last published state.
  static void publish_binary_if_changed_(binary_sensor::BinarySensor *sensor, bool value) {
    if (sensor == nullptr) return;
    if (!sensor->has_state() || sensor->state != value) sensor->publish_state(value);
  }

  /// Publish a text sensor value only if it has changed from the cached value.
  /// Uses std::string comparison to avoid redundant publishes.
  void publish_text_if_changed(text_sensor::TextSensor *sensor, std::string &cached,
                                 const std::string &value) {
    if (sensor != nullptr && value != cached) {
      sensor->publish_state(value);
      cached = value;
    }
  }
  /// Overload for const char* — uses strcmp to avoid implicit std::string construction.
  void publish_text_if_changed(text_sensor::TextSensor *sensor, std::string &cached,
                                 const char *value) {
    if (sensor == nullptr || value == nullptr) return;
    if (cached.empty() || strcmp(cached.c_str(), value) != 0) {
      cached = value;
      sensor->publish_state(cached);
    }
  }
  
  // Derived sensors (COP, delta-T, approach)
  void publish_derived_sensors(const RegisterMap &regs);

  // --- Configuration ---
  uint8_t address_{1};
  uint8_t read_retries_{2};
  GPIOPin *flow_control_pin_{nullptr};
  
  // Hardware override flags
  bool axb_override_{false};
  bool vs_drive_override_{false};
  bool iz2_override_{false};
  bool iz2_zones_override_{false};
  
  // --- State machine ---
  State state_{State::SETUP_READ_ID};
  PendingRequest pending_request_{PendingRequest::NONE};
  bool setup_complete_{false};
  uint8_t setup_retry_count_{0};
  uint8_t poll_retry_count_{0};     // Retry counter for normal poll-cycle timeouts
  static constexpr uint8_t MAX_SETUP_RETRIES = 5;
  
  // RX buffer — persists across loop() calls for incremental frame reading
  std::vector<uint8_t> rx_buffer_;
  
  // Response frame buffer — reused across loop() calls to avoid heap allocation
  std::vector<uint8_t> response_frame_;
  
  // Expected addresses for the current in-flight request
  std::vector<uint16_t> expected_addresses_;
  
  // Timing
  uint32_t last_request_time_{0};
  uint32_t error_backoff_until_{0};
  uint32_t last_successful_response_{0};
  uint32_t tx_complete_time_{0};  // millis() when TX FIFO is expected to drain
  
  // Connectivity
  binary_sensor::BinarySensor *connected_sensor_{nullptr};
  uint32_t connected_timeout_{DEFAULT_CONNECTED_TIMEOUT_MS};
  bool connected_{false};
  
  // Write queue — writes are queued and dispatched non-blockingly from IDLE.
  // Capped at MAX_PENDING_WRITES to prevent unbounded heap growth.
  static constexpr size_t MAX_PENDING_WRITES = 16;
  std::vector<std::pair<uint16_t, uint16_t>> pending_writes_;
  
  // Write cooldowns — prevent stale read-backs from reverting optimistic UI updates.
  // Initialized to ensure cooldown is INACTIVE at boot (unsigned wraparound arithmetic:
  // millis() - COOLDOWN_BOOT_INIT > WRITE_COOLDOWN_MS is true even when millis() == 0).
  static constexpr uint32_t COOLDOWN_BOOT_INIT = 0u - WRITE_COOLDOWN_MS - 1u;
  uint32_t last_mode_write_{COOLDOWN_BOOT_INIT};
  uint32_t last_setpoint_write_{COOLDOWN_BOOT_INIT};
  uint32_t last_fan_write_{COOLDOWN_BOOT_INIT};
  uint32_t last_dhw_write_{COOLDOWN_BOOT_INIT};
  uint32_t last_humidity_target_write_{COOLDOWN_BOOT_INIT};
  
  // Setup callbacks — fired once when hardware detection completes
  std::vector<std::function<void()>> setup_callbacks_;
  
  // Cached register values — flat sorted vector
  RegisterMap register_cache_;
  
  // Pre-allocated vector for register addresses
  std::vector<uint16_t> addresses_to_read_;
  
  // --- Heat pump state ---
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
  float relative_humidity_{NAN};
  bool humidifier_auto_{false};
  bool dehumidifier_auto_{false};

  // AWL version fields
  float thermostat_version_{0.0f};
  float axb_version_{0.0f};
  float iz2_version_{0.0f};

  // Hardware type detection
  BlowerType blower_type_{BlowerType::PSC};
  PumpType pump_type_{PumpType::OTHER};
  uint8_t energy_monitor_level_{0};
  
  // IZ2 Zone data
  IZ2ZoneData iz2_zones_[MAX_IZ2_ZONES];
  
  // =========================================================================
  // Sensor pointers — grouped by subsystem for readability.
  // All are set via set_*_sensor() from Python codegen; nullptr when not
  // configured in YAML.
  // =========================================================================

  // --- Core HVAC sensors ---
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
  sensor::Sensor *fault_code_sensor_{nullptr};
  sensor::Sensor *line_voltage_setting_sensor_{nullptr};
  sensor::Sensor *anti_short_cycle_sensor_{nullptr};

  // --- Energy monitoring sensors ---
  sensor::Sensor *total_watts_sensor_{nullptr};
  sensor::Sensor *compressor_watts_sensor_{nullptr};
  sensor::Sensor *blower_watts_sensor_{nullptr};
  sensor::Sensor *aux_watts_sensor_{nullptr};
  sensor::Sensor *pump_watts_sensor_{nullptr};
  sensor::Sensor *line_voltage_sensor_{nullptr};

  // --- Refrigeration sensors ---
  sensor::Sensor *discharge_pressure_sensor_{nullptr};
  sensor::Sensor *suction_pressure_sensor_{nullptr};
  sensor::Sensor *eev_open_sensor_{nullptr};
  sensor::Sensor *superheat_sensor_{nullptr};
  sensor::Sensor *fp1_sensor_{nullptr};
  sensor::Sensor *fp2_sensor_{nullptr};
  sensor::Sensor *heating_liquid_line_temp_sensor_{nullptr};
  sensor::Sensor *saturated_condenser_temp_sensor_{nullptr};
  sensor::Sensor *subcool_temp_sensor_{nullptr};
  sensor::Sensor *heat_of_extraction_sensor_{nullptr};
  sensor::Sensor *heat_of_rejection_sensor_{nullptr};

  // --- AXB / DHW sensors ---
  sensor::Sensor *waterflow_sensor_{nullptr};
  sensor::Sensor *loop_pressure_sensor_{nullptr};
  sensor::Sensor *dhw_temp_sensor_{nullptr};
  sensor::Sensor *dhw_setpoint_sensor_{nullptr};
  sensor::Sensor *blower_amps_sensor_{nullptr};
  sensor::Sensor *aux_amps_sensor_{nullptr};
  sensor::Sensor *compressor1_amps_sensor_{nullptr};
  sensor::Sensor *compressor2_amps_sensor_{nullptr};

  // --- VS Drive sensors ---
  sensor::Sensor *compressor_desired_speed_sensor_{nullptr};
  sensor::Sensor *discharge_temp_sensor_{nullptr};
  sensor::Sensor *suction_temp_sensor_{nullptr};
  sensor::Sensor *vs_drive_temp_sensor_{nullptr};
  sensor::Sensor *vs_inverter_temp_sensor_{nullptr};
  sensor::Sensor *vs_fan_speed_sensor_{nullptr};
  sensor::Sensor *vs_ambient_temp_sensor_{nullptr};
  sensor::Sensor *vs_compressor_watts_sensor_{nullptr};
  sensor::Sensor *sat_evap_discharge_temp_sensor_{nullptr};
  sensor::Sensor *vs_entering_water_temp_sensor_{nullptr};
  sensor::Sensor *vs_line_voltage_sensor_{nullptr};
  sensor::Sensor *vs_thermo_power_sensor_{nullptr};
  sensor::Sensor *vs_supply_voltage_sensor_{nullptr};
  sensor::Sensor *vs_udc_voltage_sensor_{nullptr};

  // --- ECM / blower speed sensors ---
  sensor::Sensor *aux_heat_stage_sensor_{nullptr};
  sensor::Sensor *blower_speed_sensor_{nullptr};
  sensor::Sensor *blower_only_speed_sensor_{nullptr};
  sensor::Sensor *lo_compressor_speed_sensor_{nullptr};
  sensor::Sensor *hi_compressor_speed_sensor_{nullptr};
  sensor::Sensor *aux_heat_speed_sensor_{nullptr};

  // --- Pump sensors ---
  sensor::Sensor *pump_speed_sensor_{nullptr};
  sensor::Sensor *pump_min_speed_sensor_{nullptr};
  sensor::Sensor *pump_max_speed_sensor_{nullptr};

  // --- IZ2 zone sensors ---
  sensor::Sensor *iz2_compressor_speed_sensor_{nullptr};
  sensor::Sensor *iz2_blower_speed_sensor_{nullptr};

  // --- Humidity control sensors ---
  sensor::Sensor *humidification_target_sensor_{nullptr};
  sensor::Sensor *dehumidification_target_sensor_{nullptr};

  // --- Derived / computed sensors ---
  sensor::Sensor *cop_sensor_{nullptr};
  sensor::Sensor *water_delta_t_sensor_{nullptr};
  sensor::Sensor *approach_temp_sensor_{nullptr};

  // =========================================================================
  // Binary sensors — grouped by subsystem
  // =========================================================================

  // --- System status ---
  binary_sensor::BinarySensor *compressor_sensor_{nullptr};
  binary_sensor::BinarySensor *blower_sensor_{nullptr};
  binary_sensor::BinarySensor *aux_heat_sensor_{nullptr};
  binary_sensor::BinarySensor *lockout_sensor_{nullptr};
  binary_sensor::BinarySensor *emergency_shutdown_sensor_{nullptr};
  binary_sensor::BinarySensor *load_shed_sensor_{nullptr};
  binary_sensor::BinarySensor *fan_call_sensor_{nullptr};
  binary_sensor::BinarySensor *derated_sensor_{nullptr};
  binary_sensor::BinarySensor *safe_mode_sensor_{nullptr};

  // --- Pressure switches ---
  binary_sensor::BinarySensor *lps_sensor_{nullptr};
  binary_sensor::BinarySensor *hps_sensor_{nullptr};

  // --- AXB outputs ---
  binary_sensor::BinarySensor *dhw_running_sensor_{nullptr};
  binary_sensor::BinarySensor *loop_pump_sensor_{nullptr};
  binary_sensor::BinarySensor *diverting_valve_sensor_{nullptr};

  // --- Humidity ---
  binary_sensor::BinarySensor *humidifier_running_sensor_{nullptr};
  binary_sensor::BinarySensor *dehumidifier_running_sensor_{nullptr};

  // =========================================================================
  // Text sensors
  // =========================================================================
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
  
  // --- Lockout diagnostic sensors ---
  sensor::Sensor *lockout_fault_sensor_{nullptr};
  text_sensor::TextSensor *lockout_fault_description_sensor_{nullptr};
  text_sensor::TextSensor *outputs_at_lockout_sensor_{nullptr};
  text_sensor::TextSensor *inputs_at_lockout_sensor_{nullptr};
  
  // Observer callbacks — bounded at init time by the number of sub-entities
  // configured in YAML; no runtime growth path exists.
  std::vector<std::function<void()>> listeners_;
  
  // Cached text sensor values for publish-on-change.
  // Bitmask-derived strings also cache the raw register value to avoid
  // calling bitmask_to_string() (which heap-allocates) when unchanged.
  std::string cached_mode_string_;
  std::string cached_fault_description_;
  std::string cached_hvac_mode_;
  std::string cached_fan_mode_;
  std::string cached_vs_derate_;
  uint16_t cached_vs_derate_raw_{0xFFFF};
  std::string cached_vs_safe_mode_;
  uint16_t cached_vs_safe_mode_raw_{0xFFFF};
  std::string cached_vs_alarm_;
  uint16_t cached_vs_alarm1_raw_{0xFFFF};
  uint16_t cached_vs_alarm2_raw_{0xFFFF};
  std::string cached_axb_inputs_;
  uint16_t cached_axb_inputs_raw_{0xFFFF};
  std::string cached_humidifier_mode_;
  std::string cached_dehumidifier_mode_;
  std::string cached_lockout_fault_description_;
  std::string cached_outputs_at_lockout_;
  uint16_t cached_outputs_at_lockout_raw_{0xFFFF};
  std::string cached_inputs_at_lockout_;
  uint16_t cached_inputs_at_lockout_raw_{0xFFFF};
  
  // Adaptive polling tier counter — intentionally uint8_t.
  // Wraps at 255; the modulo checks (% 6, % 60) produce correct results at all values.
  uint8_t poll_tier_counter_{0};
  
  // Cached device info
  std::string model_number_;
  std::string serial_number_;
  
  // Cached fault history string — reused across slow-tier poll cycles
  // to avoid heap allocation in loop(). Only rebuilt when content changes.
  std::string cached_fault_history_;
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
