#include "waterfurnace_aurora.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include <cstring>

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "waterfurnace_aurora";

// WaterFurnace custom function codes (from lib/aurora/modbus/slave.rb)
static const uint8_t FUNC_READ_RANGES = 0x41;      // 'A' - read multiple ranges
static const uint8_t FUNC_READ_SPECIFIC = 0x42;   // 'B' - read specific addresses
static const uint8_t FUNC_READ_HOLDING = 0x03;    // Standard read holding registers
static const uint8_t FUNC_WRITE_SINGLE = 0x06;    // Standard write single register

// REAL get_fault_description - 67 case switch statement
const char* WaterFurnaceAurora::get_fault_description(uint8_t code) {
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

const char* WaterFurnaceAurora::get_hvac_mode_string(HeatingMode mode) {
  switch (mode) {
    case HeatingMode::OFF: return "Off";
    case HeatingMode::AUTO: return "Auto";
    case HeatingMode::COOL: return "Cool";
    case HeatingMode::HEAT: return "Heat";
    case HeatingMode::EHEAT: return "Emergency Heat";
    default: return "Unknown";
  }
}

const char* WaterFurnaceAurora::get_fan_mode_string(FanMode mode) {
  switch (mode) {
    case FanMode::AUTO: return "Auto";
    case FanMode::CONTINUOUS: return "Continuous";
    case FanMode::INTERMITTENT: return "Intermittent";
    default: return "Unknown";
  }
}

std::string WaterFurnaceAurora::get_current_mode_string() {
  if (this->system_outputs_ & OUTPUT_LOCKOUT) {
    return "Lockout";
  }
  if (this->active_dehumidify_) {
    return "Dehumidify";
  }
  
  bool compressor = this->system_outputs_ & (OUTPUT_CC | OUTPUT_CC2);
  bool cooling = this->system_outputs_ & OUTPUT_RV;
  bool aux_heat = this->system_outputs_ & (OUTPUT_EH1 | OUTPUT_EH2);
  bool blower = this->system_outputs_ & OUTPUT_BLOWER;
  
  if (compressor) {
    if (cooling) return "Cooling";
    if (aux_heat) return "Heating + Aux";
    return "Heating";
  }
  if (aux_heat) return "Emergency Heat";
  if (blower) return "Fan Only";
  
  const uint16_t *asc = reg_find(this->register_cache_, registers::COMPRESSOR_ANTI_SHORT_CYCLE);
  if (asc != nullptr && *asc != 0) {
    return "Waiting";
  }
  
  return "Standby";
}

void WaterFurnaceAurora::setup() {
  ESP_LOGCONFIG(TAG, "Setting up WaterFurnace Aurora...");
  
  // Initialize RS485 flow control pin if configured
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
    this->flow_control_pin_->digital_write(false);  // Start in RX mode
    ESP_LOGD(TAG, "RS485 flow control pin configured");
  }
  
  // Auto-detect hardware (AXB, VS Drive, IZ2) unless overridden via YAML
  this->detect_hardware();
  
  // Read model and serial number
  this->read_device_info();
  
  // Set error status if no hardware detected and no overrides provided
  if (!this->has_axb_ && !this->has_vs_drive_ && !this->has_iz2_ &&
      !this->axb_override_ && !this->vs_drive_override_ && !this->iz2_override_) {
    this->status_set_error(LOG_STR("No response from heat pump - check RS-485 wiring"));
  }
  
  ESP_LOGI(TAG, "WaterFurnace Aurora setup complete");
  ESP_LOGI(TAG, "  AXB: %s%s (v%.2f, AWL: %s)", this->has_axb_ ? "detected" : "not detected",
           this->axb_override_ ? " (manual override)" : "",
           this->axb_version_, this->awl_axb() ? "yes" : "no");
  ESP_LOGI(TAG, "  VS Drive: %s%s", this->has_vs_drive_ ? "detected" : "not detected",
           this->vs_drive_override_ ? " (manual override)" : "");
  ESP_LOGI(TAG, "  IZ2: %s%s (v%.2f, AWL: %s)", this->has_iz2_ ? "detected" : "not detected",
           this->iz2_override_ ? " (manual override)" : "",
           this->iz2_version_, this->awl_iz2() ? "yes" : "no");
  if (this->has_iz2_) {
    ESP_LOGI(TAG, "  IZ2 Zones: %d%s", this->num_iz2_zones_,
             this->iz2_zones_override_ ? " (manual override)" : "");
  }
  ESP_LOGI(TAG, "  Thermostat v%.2f (AWL: %s)", this->thermostat_version_,
           this->awl_thermostat() ? "yes" : "no");
  ESP_LOGI(TAG, "  Blower: %s", this->blower_type_ == BlowerType::PSC ? "PSC" :
           this->blower_type_ == BlowerType::FIVE_SPEED ? "5-Speed" : "ECM");
  ESP_LOGI(TAG, "  Energy Monitor: %s",
           this->energy_monitor_level_ == 0 ? "None" :
           this->energy_monitor_level_ == 1 ? "Compressor Monitor" : "Energy Monitor");
  if (!this->model_number_.empty()) {
    ESP_LOGI(TAG, "  Model: %s", this->model_number_.c_str());
  }
  if (!this->serial_number_.empty()) {
    ESP_LOGI(TAG, "  Serial: %s", this->serial_number_.c_str());
  }
}

void WaterFurnaceAurora::detect_hardware() {
  // Ruby gem checks register 806 for AXB: value != 3 means present
  // COMPONENT_STATUS: 1=active, 2=added, 3=removed, 0xffff=missing
  
  // Read hardware detection registers using function 0x42
  std::vector<uint16_t> detect_addrs;
  detect_addrs.reserve(20);
  
  if (!this->axb_override_) {
    detect_addrs.push_back(registers::AXB_INSTALLED);  // 806
  }
  detect_addrs.push_back(registers::AXB_VERSION);       // 807 (always read for AWL check)
  detect_addrs.push_back(registers::THERMOSTAT_INSTALLED); // 800
  detect_addrs.push_back(registers::THERMOSTAT_VERSION);   // 801
  if (!this->iz2_override_) {
    detect_addrs.push_back(registers::IZ2_INSTALLED);  // 812
  }
  detect_addrs.push_back(registers::IZ2_VERSION);       // 813 (always read for AWL check)
  if (!this->iz2_zones_override_) {
    detect_addrs.push_back(registers::IZ2_NUM_ZONES);  // 483
  }
  // Hardware type detection registers
  detect_addrs.push_back(registers::BLOWER_TYPE);        // 404
  detect_addrs.push_back(registers::ENERGY_MONITOR);     // 412
  detect_addrs.push_back(registers::PUMP_TYPE);           // 413
  // Read register 88 (ABC program) to help detect VS drive
  if (!this->vs_drive_override_) {
    detect_addrs.push_back(88);  // ABC Program (4 registers: 88-91)
    detect_addrs.push_back(89);
    detect_addrs.push_back(90);
    detect_addrs.push_back(91);
  }
  
  if (detect_addrs.empty()) {
    ESP_LOGD(TAG, "All hardware flags set via YAML overrides, skipping auto-detection");
    return;
  }
  
  RegisterMap result;
  if (!this->read_specific_registers(detect_addrs, result)) {
    ESP_LOGW(TAG, "Failed to read hardware detection registers - will use defaults or overrides");
    ESP_LOGW(TAG, "Check RS-485 wiring and ensure the heat pump is powered on");
    return;
  }
  
  // Detect AXB (register 806): present if value != 3 (3 = removed)
  if (!this->axb_override_) {
    const uint16_t *val = reg_find(result, registers::AXB_INSTALLED);
    if (val) {
      // Per Ruby gem: COMPONENT_STATUS: 1=active, 2=added, 3=removed, 0xffff=missing
      this->has_axb_ = (*val != 3 && *val != 0xFFFF);
      ESP_LOGD(TAG, "AXB register 806 = %d -> %s", *val, this->has_axb_ ? "present" : "absent");
    } else {
      ESP_LOGW(TAG, "AXB register 806 not in response");
    }
  }
  
  // Detect IZ2 (register 812): present if value != 3 (3 = removed)
  if (!this->iz2_override_) {
    const uint16_t *val = reg_find(result, 812);
    if (val) {
      this->has_iz2_ = (*val != 3 && *val != 0xFFFF);
      ESP_LOGD(TAG, "IZ2 register 812 = %d -> %s", *val, this->has_iz2_ ? "present" : "absent");
    } else {
      ESP_LOGW(TAG, "IZ2 register 812 not in response");
    }
  }
  
  // Read IZ2 zone count (register 483) - even if has_iz2 was overridden, auto-detect zone count
  if (!this->iz2_zones_override_ && this->has_iz2_) {
    const uint16_t *val = reg_find(result, registers::IZ2_NUM_ZONES);
    if (val) {
      this->num_iz2_zones_ = std::min(static_cast<uint8_t>(*val), static_cast<uint8_t>(MAX_IZ2_ZONES));
      ESP_LOGD(TAG, "IZ2 zone count register 483 = %d -> %d zones", *val, this->num_iz2_zones_);
    } else {
      ESP_LOGW(TAG, "IZ2 zone count register 483 not in response");
    }
  }
  
  // Read AWL version registers for version-gated register selection
  const uint16_t *val_tver = reg_find(result, registers::THERMOSTAT_VERSION);
  if (val_tver) {
    this->thermostat_version_ = static_cast<float>(*val_tver) / 100.0f;
    ESP_LOGD(TAG, "Thermostat version: %.2f (raw %d)", this->thermostat_version_, *val_tver);
  }
  
  const uint16_t *val_aver = reg_find(result, registers::AXB_VERSION);
  if (val_aver) {
    this->axb_version_ = static_cast<float>(*val_aver) / 100.0f;
    ESP_LOGD(TAG, "AXB version: %.2f (raw %d)", this->axb_version_, *val_aver);
  }
  
  const uint16_t *val_iver = reg_find(result, registers::IZ2_VERSION);
  if (val_iver) {
    this->iz2_version_ = static_cast<float>(*val_iver) / 100.0f;
    ESP_LOGD(TAG, "IZ2 version: %.2f (raw %d)", this->iz2_version_, *val_iver);
  }
  
  // Detect blower type (register 404)
  const uint16_t *val_blower = reg_find(result, registers::BLOWER_TYPE);
  if (val_blower) {
    uint16_t bval = *val_blower;
    if (bval == 1 || bval == 2) {
      this->blower_type_ = static_cast<BlowerType>(bval);
    } else if (bval == 3) {
      this->blower_type_ = BlowerType::FIVE_SPEED;
    } else {
      this->blower_type_ = BlowerType::PSC;
    }
    ESP_LOGD(TAG, "Blower type register 404 = %d -> %s", bval,
             bval == 0 ? "PSC" : bval == 1 ? "ECM 208/230" : bval == 2 ? "ECM 265/277" : bval == 3 ? "5-Speed" : "Other/PSC");
  }
  
  // Detect energy monitor level (register 412)
  const uint16_t *val_energy = reg_find(result, registers::ENERGY_MONITOR);
  if (val_energy) {
    this->energy_monitor_level_ = std::min(static_cast<uint8_t>(*val_energy), static_cast<uint8_t>(2));
    ESP_LOGD(TAG, "Energy monitor register 412 = %d -> %s", *val_energy,
             this->energy_monitor_level_ == 0 ? "None" : this->energy_monitor_level_ == 1 ? "Compressor Monitor" : "Energy Monitor");
  }
  
  // Detect VS Drive via ABC Program (register 88, 4 registers = 8 chars ASCII)
  // Ruby gem: program names containing "VSP" or "SPLVS" indicate VS drive
  // e.g. "ABCVSP", "ABCVSPR", "ABCSPLVS"
  if (!this->vs_drive_override_) {
    std::vector<uint16_t> prog_regs;
    for (uint16_t r = 88; r <= 91; r++) {
      const uint16_t *val = reg_find(result, r);
      if (val) {
        prog_regs.push_back(*val);
      }
    }
    if (!prog_regs.empty()) {
      std::string program = registers_to_string(prog_regs);
      ESP_LOGD(TAG, "ABC Program: '%s'", program.c_str());
      // Check if program name indicates VS drive
      this->has_vs_drive_ = (program.find("VSP") != std::string::npos || 
                              program.find("SPLVS") != std::string::npos);
      ESP_LOGD(TAG, "VS Drive detection from program '%s' -> %s", 
               program.c_str(), this->has_vs_drive_ ? "present" : "absent");
    }
    
    // Fallback: if program name didn't match, try reading VS drive-specific 
    // registers that only have meaningful values when a VS drive is present.
    // Register 3322 (VS discharge pressure) and 3325 (VS discharge temp) should 
    // have non-zero values when a VS drive is communicating.
    if (!this->has_vs_drive_) {
      std::vector<uint16_t> vs_probe = {3001, 3322, 3325};
      RegisterMap vs_result;
      if (this->read_specific_registers(vs_probe, vs_result)) {
        const uint16_t *v_speed = reg_find(vs_result, 3001);
        const uint16_t *v_press = reg_find(vs_result, 3322);
        const uint16_t *v_temp = reg_find(vs_result, 3325);
        // Consider VS drive present if we got any non-zero VS-specific values
        bool has_data = false;
        if (v_press && *v_press != 0) has_data = true;
        if (v_temp && *v_temp != 0) has_data = true;
        if (v_speed && *v_speed != 0) has_data = true;
        if (has_data) {
          this->has_vs_drive_ = true;
          ESP_LOGD(TAG, "VS Drive detected via register probe (speed=%d, pressure=%d, temp=%d)",
                   v_speed ? *v_speed : 0,
                   v_press ? *v_press : 0,
                   v_temp ? *v_temp : 0);
        }
      } else {
        ESP_LOGD(TAG, "VS Drive register probe failed - no VS drive");
      }
    }
  }
}

void WaterFurnaceAurora::read_device_info() {
  // Read model number (registers 92-103, 12 registers = 24 chars ASCII)
  std::vector<uint16_t> model_result;
  if (this->read_holding_registers(registers::MODEL_NUMBER, 12, model_result)) {
    this->model_number_ = registers_to_string(model_result);
    if (this->model_number_sensor_ != nullptr && !this->model_number_.empty()) {
      this->model_number_sensor_->publish_state(this->model_number_);
    }
    ESP_LOGD(TAG, "Model number: '%s'", this->model_number_.c_str());
  } else {
    ESP_LOGW(TAG, "Failed to read model number");
  }
  
  // Read serial number (registers 105-109, 5 registers = 10 chars ASCII)
  std::vector<uint16_t> serial_result;
  if (this->read_holding_registers(registers::SERIAL_NUMBER, 5, serial_result)) {
    this->serial_number_ = registers_to_string(serial_result);
    if (this->serial_number_sensor_ != nullptr && !this->serial_number_.empty()) {
      this->serial_number_sensor_->publish_state(this->serial_number_);
    }
    ESP_LOGD(TAG, "Serial number: '%s'", this->serial_number_.c_str());
  } else {
    ESP_LOGW(TAG, "Failed to read serial number");
  }
}

void WaterFurnaceAurora::loop() {
  // Nothing to do in loop - all communication happens in update()
}

void WaterFurnaceAurora::update() {
  this->poll_tier_counter_++;
  ESP_LOGD(TAG, "Updating WaterFurnace Aurora data (cycle %d)...", this->poll_tier_counter_);
  this->refresh_all_data();
}

void WaterFurnaceAurora::dump_config() {
  ESP_LOGCONFIG(TAG, "WaterFurnace Aurora:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Flow Control Pin: %s", this->flow_control_pin_ != nullptr ? "configured" : "not configured");
  ESP_LOGCONFIG(TAG, "  Read Retries: %d", this->read_retries_);
  if (!this->model_number_.empty()) {
    ESP_LOGCONFIG(TAG, "  Model: %s", this->model_number_.c_str());
  }
  if (!this->serial_number_.empty()) {
    ESP_LOGCONFIG(TAG, "  Serial: %s", this->serial_number_.c_str());
  }
  ESP_LOGCONFIG(TAG, "  AXB: %s%s (v%.2f, AWL: %s)", this->has_axb_ ? "yes" : "no",
                this->axb_override_ ? " (override)" : "",
                this->axb_version_, this->awl_axb() ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  VS Drive: %s%s", this->has_vs_drive_ ? "yes" : "no",
                this->vs_drive_override_ ? " (override)" : "");
  ESP_LOGCONFIG(TAG, "  IZ2: %s%s (v%.2f, AWL: %s)", this->has_iz2_ ? "yes" : "no",
                this->iz2_override_ ? " (override)" : "",
                this->iz2_version_, this->awl_iz2() ? "yes" : "no");
  if (this->has_iz2_) {
    ESP_LOGCONFIG(TAG, "  IZ2 Zones: %d%s", this->num_iz2_zones_,
                  this->iz2_zones_override_ ? " (override)" : "");
  }
  ESP_LOGCONFIG(TAG, "  Blower: %s", this->blower_type_ == BlowerType::PSC ? "PSC" :
                this->blower_type_ == BlowerType::FIVE_SPEED ? "5-Speed" : "ECM");
  ESP_LOGCONFIG(TAG, "  Energy Monitor: %s",
                this->energy_monitor_level_ == 0 ? "None" :
                this->energy_monitor_level_ == 1 ? "Compressor Monitor" : "Energy Monitor");
}

const IZ2ZoneData& WaterFurnaceAurora::get_zone_data(uint8_t zone_number) const {
  if (zone_number < 1 || zone_number > MAX_IZ2_ZONES) {
    ESP_LOGW(TAG, "Invalid zone_number %d in get_zone_data (expected 1-%d)", zone_number, MAX_IZ2_ZONES);
    return iz2_zones_[0];  // Safe fallback
  }
  return iz2_zones_[zone_number - 1];
}

bool WaterFurnaceAurora::validate_zone_number(uint8_t zone_number) const {
  if (zone_number < 1 || zone_number > MAX_IZ2_ZONES) {
    ESP_LOGW(TAG, "Invalid zone number %d (1-%d)", zone_number, MAX_IZ2_ZONES);
    return false;
  }
  return true;
}

// REAL refresh_all_data - using class member to avoid stack overflow
// Adaptive polling tiers to reduce RS-485 bus contention:
//   Tier 0 (every cycle, ~5s): System outputs, temps, speeds, watts, pressures
//   Tier 1 (every 6th cycle, ~30s): Setpoints, mode config, humidistat, DHW settings
//   Tier 2 (every 60th cycle, ~5min): Fault history
void WaterFurnaceAurora::refresh_all_data() {
  bool medium_poll = (this->poll_tier_counter_ % 6) == 0;
  bool slow_poll = (this->poll_tier_counter_ % 60) == 0;
  
  // Use class member instead of local variable to avoid stack overflow
  // Clear and reuse the pre-allocated vector
  this->addresses_to_read_.clear();
  if (this->addresses_to_read_.capacity() < 100) {
    this->addresses_to_read_.reserve(100);
  }
  
  // === TIER 0: Fast registers — every cycle (~5s) ===
  // These change in real-time during heat pump operation
  
  // Core system state
  this->addresses_to_read_.push_back(registers::COMPRESSOR_ANTI_SHORT_CYCLE);  // 6
  this->addresses_to_read_.push_back(registers::LAST_FAULT);                    // 25
  this->addresses_to_read_.push_back(registers::SYSTEM_OUTPUTS);                // 30
  this->addresses_to_read_.push_back(registers::SYSTEM_STATUS);                 // 31
  
  // Temperatures (change continuously during operation)
  this->addresses_to_read_.push_back(registers::FP1_TEMP);                      // 19
  this->addresses_to_read_.push_back(registers::FP2_TEMP);                      // 20
  this->addresses_to_read_.push_back(registers::AMBIENT_TEMP);                  // 502
  
  if (this->awl_axb()) {
    this->addresses_to_read_.push_back(registers::ENTERING_AIR_AWL);            // 740
    this->addresses_to_read_.push_back(registers::LEAVING_AIR);                 // 900
  } else {
    this->addresses_to_read_.push_back(registers::ENTERING_AIR);                // 567
  }
  if (this->awl_communicating()) {
    this->addresses_to_read_.push_back(registers::RELATIVE_HUMIDITY);           // 741
    this->addresses_to_read_.push_back(registers::OUTDOOR_TEMP);                // 742
  }
  if (this->has_axb_) {
    this->addresses_to_read_.push_back(registers::AXB_OUTPUTS);                 // 1104
    this->addresses_to_read_.push_back(registers::LEAVING_WATER);               // 1110
    this->addresses_to_read_.push_back(registers::ENTERING_WATER);              // 1111
    this->addresses_to_read_.push_back(registers::WATERFLOW);                   // 1117
  }
  
  // Refrigeration monitoring (changes with compressor operation)
  if (this->refrigeration_monitoring()) {
    this->addresses_to_read_.push_back(registers::HEATING_LIQUID_LINE_TEMP);    // 1109
    this->addresses_to_read_.push_back(registers::SATURATED_CONDENSER_TEMP);    // 1134
    this->addresses_to_read_.push_back(registers::SUBCOOL_HEATING);             // 1135
    this->addresses_to_read_.push_back(registers::SUBCOOL_COOLING);             // 1136
    this->addresses_to_read_.push_back(registers::HEAT_OF_EXTRACTION);          // 1154
    this->addresses_to_read_.push_back(registers::HEAT_OF_EXTRACTION + 1);      // 1155
    this->addresses_to_read_.push_back(registers::HEAT_OF_REJECTION);           // 1156
    this->addresses_to_read_.push_back(registers::HEAT_OF_REJECTION + 1);       // 1157
  }
  
  // Energy monitoring (changes with load)
  if (this->energy_monitoring()) {
    this->addresses_to_read_.push_back(registers::LINE_VOLTAGE);                // 16
    this->addresses_to_read_.push_back(registers::COMPRESSOR_WATTS);            // 1146
    this->addresses_to_read_.push_back(registers::COMPRESSOR_WATTS + 1);        // 1147
    this->addresses_to_read_.push_back(registers::BLOWER_WATTS);                // 1148
    this->addresses_to_read_.push_back(registers::BLOWER_WATTS + 1);            // 1149
    this->addresses_to_read_.push_back(registers::AUX_WATTS);                   // 1150
    this->addresses_to_read_.push_back(registers::AUX_WATTS + 1);              // 1151
    this->addresses_to_read_.push_back(registers::TOTAL_WATTS);                 // 1152
    this->addresses_to_read_.push_back(registers::TOTAL_WATTS + 1);             // 1153
    this->addresses_to_read_.push_back(registers::PUMP_WATTS);                  // 1164
    this->addresses_to_read_.push_back(registers::PUMP_WATTS + 1);              // 1165
  }
  
  // Blower speed (changes with operating state)
  if (this->is_ecm_blower()) {
    this->addresses_to_read_.push_back(registers::ECM_SPEED);                   // 344
  } else if (this->blower_type_ == BlowerType::FIVE_SPEED) {
    this->addresses_to_read_.push_back(registers::ECM_SPEED);                   // 344
  }
  
  // VS Drive real-time telemetry
  if (this->has_vs_drive_) {
    this->addresses_to_read_.push_back(registers::ACTIVE_DEHUMIDIFY);            // 362
    this->addresses_to_read_.push_back(registers::COMPRESSOR_SPEED_DESIRED);    // 3000
    this->addresses_to_read_.push_back(registers::COMPRESSOR_SPEED_ACTUAL);     // 3001
    this->addresses_to_read_.push_back(registers::VS_DISCHARGE_PRESSURE);       // 3322
    this->addresses_to_read_.push_back(registers::VS_SUCTION_PRESSURE);         // 3323
    this->addresses_to_read_.push_back(registers::VS_DISCHARGE_TEMP);           // 3325
    this->addresses_to_read_.push_back(registers::VS_AMBIENT_TEMP);             // 3326
    this->addresses_to_read_.push_back(registers::VS_DRIVE_TEMP);               // 3327
    this->addresses_to_read_.push_back(registers::VS_INVERTER_TEMP);            // 3522
    this->addresses_to_read_.push_back(registers::VS_COMPRESSOR_WATTS);          // 3422
    this->addresses_to_read_.push_back(registers::VS_COMPRESSOR_WATTS + 1);     // 3423
    this->addresses_to_read_.push_back(registers::VS_FAN_SPEED);                // 3524
    this->addresses_to_read_.push_back(registers::VS_EEV_OPEN);                 // 3808
    this->addresses_to_read_.push_back(registers::VS_SUCTION_TEMP);             // 3903
    this->addresses_to_read_.push_back(registers::VS_SAT_EVAP_DISCHARGE_TEMP);  // 3905
    this->addresses_to_read_.push_back(registers::VS_SUPERHEAT_TEMP);           // 3906
  }
  
  // VS pump speed (changes with load)
  if (this->has_axb_ && this->awl_axb()) {
    this->addresses_to_read_.push_back(registers::VS_PUMP_SPEED);               // 325
  }
  
  // IZ2 Zone data (zone temps and call states change in real-time)
  if (this->has_iz2_ && this->num_iz2_zones_ > 0) {
    this->addresses_to_read_.push_back(registers::IZ2_OUTDOOR_TEMP);            // 31003
    this->addresses_to_read_.push_back(registers::IZ2_DEMAND);                  // 31005
    
    for (uint8_t zone = 1; zone <= this->num_iz2_zones_; zone++) {
      this->addresses_to_read_.push_back(registers::IZ2_AMBIENT_BASE + ((zone - 1) * 3));
      this->addresses_to_read_.push_back(registers::IZ2_CONFIG1_BASE + ((zone - 1) * 3));
      this->addresses_to_read_.push_back(registers::IZ2_CONFIG2_BASE + ((zone - 1) * 3));
      this->addresses_to_read_.push_back(registers::IZ2_CONFIG3_BASE + ((zone - 1) * 3));
    }
  }
  
  // === TIER 1: Medium registers — every 6th cycle (~30s) ===
  // These only change on user action or system configuration changes
  if (medium_poll) {
    ESP_LOGD(TAG, "Medium poll cycle — reading setpoints, config, humidistat");
    
    // Line voltage setting (system config, rarely changes)
    this->addresses_to_read_.push_back(registers::LINE_VOLTAGE_SETTING);        // 112
    
    // Setpoints (change on user action)
    this->addresses_to_read_.push_back(registers::HEATING_SETPOINT);            // 745
    this->addresses_to_read_.push_back(registers::COOLING_SETPOINT);            // 746
    
    // Mode configuration (change on user action)
    this->addresses_to_read_.push_back(registers::FAN_CONFIG);                  // 12005
    this->addresses_to_read_.push_back(registers::HEATING_MODE_READ);           // 12006
    
    // DHW settings (change on user action)
    if (this->has_axb_) {
      this->addresses_to_read_.push_back(registers::DHW_ENABLED);               // 400
      this->addresses_to_read_.push_back(registers::DHW_SETPOINT);              // 401
      this->addresses_to_read_.push_back(registers::DHW_TEMP);                  // 1114
      this->addresses_to_read_.push_back(registers::LOOP_PRESSURE);             // 1119
      this->addresses_to_read_.push_back(registers::AXB_INPUTS);                // 1103
    }
    
    // Blower ECM speed settings (configurable, not real-time readings)
    if (this->is_ecm_blower()) {
      this->addresses_to_read_.push_back(registers::BLOWER_ONLY_SPEED);         // 340
      this->addresses_to_read_.push_back(registers::LO_COMPRESSOR_ECM_SPEED);   // 341
      this->addresses_to_read_.push_back(registers::HI_COMPRESSOR_ECM_SPEED);   // 342
      this->addresses_to_read_.push_back(registers::AUX_HEAT_ECM_SPEED);        // 347
    }
    
    // VS Pump min/max settings
    if (this->has_axb_) {
      this->addresses_to_read_.push_back(registers::VS_PUMP_MIN);               // 321
      this->addresses_to_read_.push_back(registers::VS_PUMP_MAX);               // 322
    }
    
    // VS Drive status flags (change on fault events, not every cycle)
    if (this->has_vs_drive_) {
      this->addresses_to_read_.push_back(registers::VS_DERATE);                 // 214
      this->addresses_to_read_.push_back(registers::VS_SAFE_MODE);              // 216
      this->addresses_to_read_.push_back(registers::VS_ALARM1);                 // 217
      this->addresses_to_read_.push_back(registers::VS_ALARM2);                 // 218
    }
    
    // Humidistat — IZ2 uses different registers per Ruby gem's humidistat.rb
    if (this->has_iz2_ && this->awl_communicating()) {
      this->addresses_to_read_.push_back(registers::IZ2_HUMIDISTAT_SETTINGS);   // 21114
      this->addresses_to_read_.push_back(registers::IZ2_HUMIDISTAT_MODE);       // 31109
      this->addresses_to_read_.push_back(registers::IZ2_HUMIDISTAT_TARGETS);    // 31110
    } else {
      this->addresses_to_read_.push_back(registers::HUMIDISTAT_SETTINGS);       // 12309
      this->addresses_to_read_.push_back(registers::HUMIDISTAT_TARGETS);        // 12310
    }
  }
  
  // Read all registers using custom 'B' function
  RegisterMap result;
  if (!this->read_specific_registers(this->addresses_to_read_, result)) {
    ESP_LOGW(TAG, "Failed to read registers from Aurora");
    this->status_set_warning(LOG_STR("Communication error - retrying"));
    return;
  }
  
  // Clear any previous error/warning on successful communication
  this->status_clear_warning();
  this->status_clear_error();
  
  // Store in cache — move instead of copy to avoid double allocation
  this->register_cache_ = std::move(result);
  // Use register_cache_ from here (result is moved-from)
  const RegisterMap &regs = this->register_cache_;
  
  // Parse and publish system status
  {
    const uint16_t *val = reg_find(regs, registers::LAST_FAULT);
    if (val) {
      this->current_fault_ = *val & 0x7FFF;
      this->locked_out_ = (*val & 0x8000) != 0;
      
      if (this->fault_code_sensor_ != nullptr) {
        this->fault_code_sensor_->publish_state(this->current_fault_);
      }
      this->publish_text_if_changed(this->fault_description_sensor_, this->cached_fault_description_,
                                     get_fault_description(this->current_fault_));
      if (this->lockout_sensor_ != nullptr) {
        this->lockout_sensor_->publish_state(this->locked_out_);
      }
    }
  }
  
  // System outputs (register 30)
  {
    const uint16_t *val = reg_find(regs, registers::SYSTEM_OUTPUTS);
    if (val) {
      this->system_outputs_ = *val;
      
      if (this->compressor_sensor_ != nullptr) {
        this->compressor_sensor_->publish_state((this->system_outputs_ & (OUTPUT_CC | OUTPUT_CC2)) != 0);
      }
      if (this->blower_sensor_ != nullptr) {
        this->blower_sensor_->publish_state((this->system_outputs_ & OUTPUT_BLOWER) != 0);
      }
      if (this->aux_heat_sensor_ != nullptr) {
        this->aux_heat_sensor_->publish_state((this->system_outputs_ & (OUTPUT_EH1 | OUTPUT_EH2)) != 0);
      }
      // Aux heat stage: 2 if EH2 set, 1 if only EH1 set, 0 if neither (matches Ruby gem aux_heat.rb)
      if (this->aux_heat_stage_sensor_ != nullptr) {
        uint8_t stage = (this->system_outputs_ & OUTPUT_EH2) ? 2
                      : (this->system_outputs_ & OUTPUT_EH1) ? 1
                      : 0;
        this->aux_heat_stage_sensor_->publish_state(stage);
      }
    }
  }
  
  // System status (register 31)
  {
    const uint16_t *val = reg_find(regs, registers::SYSTEM_STATUS);
    if (val) {
      uint16_t status = *val;
      
      if (this->lps_sensor_ != nullptr) {
        this->lps_sensor_->publish_state((status & STATUS_LPS) != 0);
      }
      if (this->hps_sensor_ != nullptr) {
        this->hps_sensor_->publish_state((status & STATUS_HPS) != 0);
      }
      if (this->emergency_shutdown_sensor_ != nullptr) {
        this->emergency_shutdown_sensor_->publish_state((status & STATUS_EMERGENCY_SHUTDOWN) != 0);
      }
      if (this->load_shed_sensor_ != nullptr) {
        this->load_shed_sensor_->publish_state((status & STATUS_LOAD_SHED) != 0);
      }
    }
  }
  
  // AXB inputs (register 1103)
  {
    const uint16_t *val = reg_find(regs, registers::AXB_INPUTS);
    if (val) {
      this->publish_text_if_changed(this->axb_inputs_sensor_, this->cached_axb_inputs_,
                                     get_axb_inputs_string(*val));
    }
  }
  
  // AXB outputs (register 1104)
  {
    const uint16_t *val = reg_find(regs, registers::AXB_OUTPUTS);
    if (val) {
      this->axb_outputs_ = *val;
      
      if (this->dhw_running_sensor_ != nullptr) {
        this->dhw_running_sensor_->publish_state((this->axb_outputs_ & AXB_OUTPUT_DHW) != 0);
      }
      if (this->loop_pump_sensor_ != nullptr) {
        this->loop_pump_sensor_->publish_state((this->axb_outputs_ & AXB_OUTPUT_LOOP_PUMP) != 0);
      }
    }
  }
  
  // Active dehumidify (register 362)
  {
    const uint16_t *val = reg_find(regs, registers::ACTIVE_DEHUMIDIFY);
    if (val) {
      this->active_dehumidify_ = (*val != 0);
    }
  }
  
  // Current mode
  this->publish_text_if_changed(this->current_mode_sensor_, this->cached_mode_string_,
                                 this->get_current_mode_string());
  
  // Temperatures — use publish helpers for simple sensor publications
  {
    const uint16_t *val_amb = reg_find(regs, registers::AMBIENT_TEMP);
    if (val_amb) {
      this->ambient_temp_ = to_signed_tenths(*val_amb);
      if (this->ambient_temp_sensor_ != nullptr) {
        this->ambient_temp_sensor_->publish_state(this->ambient_temp_);
      }
    }
  }
  
  uint16_t entering_air_reg = this->awl_axb() ? registers::ENTERING_AIR_AWL : registers::ENTERING_AIR;
  this->publish_sensor_signed_tenths(regs, entering_air_reg, this->entering_air_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::LEAVING_AIR, this->leaving_air_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::OUTDOOR_TEMP, this->outdoor_temp_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::LEAVING_WATER, this->leaving_water_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::ENTERING_WATER, this->entering_water_sensor_);
  
  // Humidity
  this->publish_sensor(regs, registers::RELATIVE_HUMIDITY, this->humidity_sensor_);
  
  // Setpoints — also update cached state for climate entities
  {
    const uint16_t *val_hsp = reg_find(regs, registers::HEATING_SETPOINT);
    if (val_hsp) {
      this->heating_setpoint_ = to_tenths(*val_hsp);
      if (this->heating_setpoint_sensor_ != nullptr) {
        this->heating_setpoint_sensor_->publish_state(this->heating_setpoint_);
      }
    }
  }
  {
    const uint16_t *val_csp = reg_find(regs, registers::COOLING_SETPOINT);
    if (val_csp) {
      this->cooling_setpoint_ = to_tenths(*val_csp);
      if (this->cooling_setpoint_sensor_ != nullptr) {
        this->cooling_setpoint_sensor_->publish_state(this->cooling_setpoint_);
      }
    }
  }
  
  // DHW
  {
    const uint16_t *val_dhw = reg_find(regs, registers::DHW_ENABLED);
    if (val_dhw) {
      this->dhw_enabled_ = (*val_dhw != 0);
    }
  }
  
  {
    const uint16_t *val_dhwsp = reg_find(regs, registers::DHW_SETPOINT);
    if (val_dhwsp) {
      this->dhw_setpoint_ = to_tenths(*val_dhwsp);
      if (this->dhw_setpoint_sensor_ != nullptr) {
        this->dhw_setpoint_sensor_->publish_state(this->dhw_setpoint_);
      }
    }
  }
  {
    const uint16_t *val_dhwt = reg_find(regs, registers::DHW_TEMP);
    if (val_dhwt) {
      this->dhw_temp_ = to_signed_tenths(*val_dhwt);
      if (this->dhw_temp_sensor_ != nullptr) {
        this->dhw_temp_sensor_->publish_state(this->dhw_temp_);
      }
    }
  }
  
  // HVAC mode (register 12006)
  {
    const uint16_t *val_mode = reg_find(regs, registers::HEATING_MODE_READ);
    if (val_mode) {
      uint8_t mode_val = (*val_mode >> 8) & 0x07;
      this->hvac_mode_ = static_cast<HeatingMode>(mode_val);
      this->publish_text_if_changed(this->hvac_mode_sensor_, this->cached_hvac_mode_,
                                     get_hvac_mode_string(this->hvac_mode_));
    }
  }
  
  // Fan mode (register 12005)
  {
    const uint16_t *val_fan = reg_find(regs, registers::FAN_CONFIG);
    if (val_fan) {
      uint16_t config = *val_fan;
      if (config & 0x80) {
        this->fan_mode_ = FanMode::CONTINUOUS;
      } else if (config & 0x100) {
        this->fan_mode_ = FanMode::INTERMITTENT;
      } else {
        this->fan_mode_ = FanMode::AUTO;
      }
      this->publish_text_if_changed(this->fan_mode_sensor_, this->cached_fan_mode_,
                                     get_fan_mode_string(this->fan_mode_));
    }
  }
  
  // Line voltage
  this->publish_sensor(regs, registers::LINE_VOLTAGE, this->line_voltage_sensor_);
  
  // Power — 32-bit unsigned values from register pairs
  this->publish_sensor_uint32(regs, registers::TOTAL_WATTS, this->total_watts_sensor_);
  this->publish_sensor_uint32(regs, registers::COMPRESSOR_WATTS, this->compressor_watts_sensor_);
  this->publish_sensor_uint32(regs, registers::BLOWER_WATTS, this->blower_watts_sensor_);
  this->publish_sensor_uint32(regs, registers::AUX_WATTS, this->aux_watts_sensor_);
  this->publish_sensor_uint32(regs, registers::PUMP_WATTS, this->pump_watts_sensor_);
  
  // Loop
  this->publish_sensor_tenths(regs, registers::WATERFLOW, this->waterflow_sensor_);
  {
    const uint16_t *val_lp = reg_find(regs, registers::LOOP_PRESSURE);
    if (val_lp && *val_lp < 10000 && this->loop_pressure_sensor_ != nullptr) {
      this->loop_pressure_sensor_->publish_state(to_tenths(*val_lp));
    }
  }
  
  // VS Drive
  this->publish_sensor(regs, registers::COMPRESSOR_SPEED_ACTUAL, this->compressor_speed_sensor_);
  this->publish_sensor_tenths(regs, registers::VS_DISCHARGE_PRESSURE, this->discharge_pressure_sensor_);
  this->publish_sensor_tenths(regs, registers::VS_SUCTION_PRESSURE, this->suction_pressure_sensor_);
  this->publish_sensor(regs, registers::VS_EEV_OPEN, this->eev_open_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_SUPERHEAT_TEMP, this->superheat_sensor_);
  
  // Additional VS Drive sensors
  this->publish_sensor(regs, registers::COMPRESSOR_SPEED_DESIRED, this->compressor_desired_speed_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_DISCHARGE_TEMP, this->discharge_temp_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_SUCTION_TEMP, this->suction_temp_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_DRIVE_TEMP, this->vs_drive_temp_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_INVERTER_TEMP, this->vs_inverter_temp_sensor_);
  this->publish_sensor(regs, registers::VS_FAN_SPEED, this->vs_fan_speed_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_AMBIENT_TEMP, this->vs_ambient_temp_sensor_);
  this->publish_sensor_uint32(regs, registers::VS_COMPRESSOR_WATTS, this->vs_compressor_watts_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_SAT_EVAP_DISCHARGE_TEMP, this->sat_evap_discharge_temp_sensor_);
  
  // VS Drive derate status (register 214)
  {
    const uint16_t *val = reg_find(regs, registers::VS_DERATE);
    if (val) {
      this->publish_text_if_changed(this->vs_derate_sensor_, this->cached_vs_derate_,
                                     get_vs_derate_string(*val));
    }
  }
  
  // VS Drive safe mode status (register 216)
  {
    const uint16_t *val = reg_find(regs, registers::VS_SAFE_MODE);
    if (val) {
      this->publish_text_if_changed(this->vs_safe_mode_sensor_, this->cached_vs_safe_mode_,
                                     get_vs_safe_mode_string(*val));
    }
  }
  
  // VS Drive alarm status (registers 217-218)
  {
    const uint16_t *val_a1 = reg_find(regs, registers::VS_ALARM1);
    const uint16_t *val_a2 = reg_find(regs, registers::VS_ALARM2);
    if (val_a1 && val_a2) {
      this->publish_text_if_changed(this->vs_alarm_sensor_, this->cached_vs_alarm_,
                                     get_vs_alarm_string(*val_a1, *val_a2));
    }
  }
  
  // FP1/FP2 refrigerant temperatures
  this->publish_sensor_signed_tenths(regs, registers::FP1_TEMP, this->fp1_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::FP2_TEMP, this->fp2_sensor_);
  
  // Line voltage setting and anti-short-cycle
  this->publish_sensor(regs, registers::LINE_VOLTAGE_SETTING, this->line_voltage_setting_sensor_);
  this->publish_sensor(regs, registers::COMPRESSOR_ANTI_SHORT_CYCLE, this->anti_short_cycle_sensor_);
  
  // Blower/ECM sensors
  this->publish_sensor(regs, registers::ECM_SPEED, this->blower_speed_sensor_);
  this->publish_sensor(regs, registers::BLOWER_ONLY_SPEED, this->blower_only_speed_sensor_);
  this->publish_sensor(regs, registers::LO_COMPRESSOR_ECM_SPEED, this->lo_compressor_speed_sensor_);
  this->publish_sensor(regs, registers::HI_COMPRESSOR_ECM_SPEED, this->hi_compressor_speed_sensor_);
  this->publish_sensor(regs, registers::AUX_HEAT_ECM_SPEED, this->aux_heat_speed_sensor_);
  
  // VS Pump sensors
  this->publish_sensor(regs, registers::VS_PUMP_SPEED, this->pump_speed_sensor_);
  this->publish_sensor(regs, registers::VS_PUMP_MIN, this->pump_min_speed_sensor_);
  this->publish_sensor(regs, registers::VS_PUMP_MAX, this->pump_max_speed_sensor_);
  
  // Refrigeration monitoring sensors
  this->publish_sensor_signed_tenths(regs, registers::HEATING_LIQUID_LINE_TEMP, this->heating_liquid_line_temp_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::SATURATED_CONDENSER_TEMP, this->saturated_condenser_temp_sensor_);
  
  // Subcool temperature
  bool is_cooling = (this->system_outputs_ & OUTPUT_RV) != 0;
  uint16_t subcool_reg = is_cooling ? registers::SUBCOOL_COOLING : registers::SUBCOOL_HEATING;
  {
    const uint16_t *val = reg_find(regs, subcool_reg);
    if (val && this->subcool_temp_sensor_ != nullptr) {
      this->subcool_temp_sensor_->publish_state(to_signed_tenths(*val));
    }
  }
  
  // Heat of extraction/rejection (32-bit, signed)
  this->publish_sensor_int32(regs, registers::HEAT_OF_EXTRACTION, this->heat_of_extraction_sensor_);
  this->publish_sensor_int32(regs, registers::HEAT_OF_REJECTION, this->heat_of_rejection_sensor_);
  
  // Humidifier/Dehumidifier status
  if (this->humidifier_running_sensor_ != nullptr) {
    bool humidifier_running = (this->system_outputs_ & OUTPUT_ACCESSORY) != 0;
    this->humidifier_running_sensor_->publish_state(humidifier_running);
  }
  
  if (this->dehumidifier_running_sensor_ != nullptr) {
    bool dehumidifier_running = this->active_dehumidify_ || 
                                 ((this->axb_outputs_ & 0x10) != 0);
    this->dehumidifier_running_sensor_->publish_state(dehumidifier_running);
  }
  
  // Humidistat — mode and targets (IZ2 vs non-IZ2 per Ruby gem humidistat.rb)
  {
    // Determine which registers to read based on IZ2 presence
    uint16_t mode_reg = (this->has_iz2_ && this->awl_communicating())
                            ? registers::IZ2_HUMIDISTAT_MODE      // 31109
                            : registers::HUMIDISTAT_SETTINGS;     // 12309
    uint16_t target_reg = (this->has_iz2_ && this->awl_communicating())
                              ? registers::IZ2_HUMIDISTAT_TARGETS // 31110
                              : registers::HUMIDISTAT_TARGETS;    // 12310

    // Parse mode bitmask: 0x8000 = auto humidification, 0x4000 = auto dehumidification
    const uint16_t *val_mode = reg_find(regs, mode_reg);
    if (val_mode) {
      this->publish_text_if_changed(this->humidifier_mode_sensor_, this->cached_humidifier_mode_,
                                     (*val_mode & 0x8000) ? "Auto" : "Manual");
      this->publish_text_if_changed(this->dehumidifier_mode_sensor_, this->cached_dehumidifier_mode_,
                                     (*val_mode & 0x4000) ? "Auto" : "Manual");
    }

    // Parse targets: high byte = humidification, low byte = dehumidification
    const uint16_t *val_targets = reg_find(regs, target_reg);
    if (val_targets) {
      if (this->humidification_target_sensor_ != nullptr) {
        this->humidification_target_sensor_->publish_state((*val_targets >> 8) & 0xFF);
      }
      if (this->dehumidification_target_sensor_ != nullptr) {
        this->dehumidification_target_sensor_->publish_state(*val_targets & 0xFF);
      }
    }
  }
  
  // === TIER 2: Slow — fault history every ~5 minutes ===
  if (slow_poll && this->fault_history_sensor_ != nullptr) {
    this->read_fault_history();
  }
  
  // Parse IZ2 zone data
  if (this->has_iz2_ && this->num_iz2_zones_ > 0) {
    for (uint8_t zone = 1; zone <= this->num_iz2_zones_; zone++) {
      IZ2ZoneData& zone_data = this->iz2_zones_[zone - 1];
      
      // Ambient temperature
      const uint16_t *val_amb = reg_find(regs, registers::IZ2_AMBIENT_BASE + ((zone - 1) * 3));
      if (val_amb) {
        zone_data.ambient_temperature = to_signed_tenths(*val_amb);
      }
      
      // Config1
      const uint16_t *val_c1 = reg_find(regs, registers::IZ2_CONFIG1_BASE + ((zone - 1) * 3));
      if (val_c1) {
        uint16_t config1 = *val_c1;
        
        if (config1 & 0x80) {
          zone_data.target_fan_mode = FanMode::CONTINUOUS;
        } else if (config1 & 0x100) {
          zone_data.target_fan_mode = FanMode::INTERMITTENT;
        } else {
          zone_data.target_fan_mode = FanMode::AUTO;
        }
        
        zone_data.fan_on_time = ((config1 >> 9) & 0x7) * 5;
        zone_data.fan_off_time = (((config1 >> 12) & 0x7) + 1) * 5;
        zone_data.cooling_setpoint = ((config1 & 0x7e) >> 1) + 36.0f;
      }
      
      // Config2
      const uint16_t *val_c2 = reg_find(regs, registers::IZ2_CONFIG2_BASE + ((zone - 1) * 3));
      if (val_c2 && val_c1) {
        uint16_t config2 = *val_c2;
        uint16_t config1 = *val_c1;
        
        uint8_t call_val = (config2 >> 1) & 0x7;
        zone_data.current_call = static_cast<ZoneCall>(call_val);
        
        uint8_t mode_val = (config2 >> 8) & 0x3;
        zone_data.target_mode = static_cast<HeatingMode>(mode_val);
        
        zone_data.damper_open = (config2 & 0x10) != 0;
        
        uint8_t carry = config1 & 0x01;
        zone_data.heating_setpoint = ((carry << 5) | ((config2 & 0xf800) >> 11)) + 36.0f;
      }
      
      // Config3
      const uint16_t *val_c3 = reg_find(regs, registers::IZ2_CONFIG3_BASE + ((zone - 1) * 3));
      if (val_c3) {
        uint16_t config3 = *val_c3;
        
        zone_data.priority = (config3 & 0x20) ? ZonePriority::ECONOMY : ZonePriority::COMFORT;
        
        uint8_t size_val = (config3 >> 3) & 0x3;
        zone_data.size = static_cast<ZoneSize>(size_val);
        
        zone_data.normalized_size = (config3 >> 8) & 0xFF;
      }
      
      ESP_LOGV(TAG, "Zone %d: temp=%.1f, heat_sp=%.1f, cool_sp=%.1f, mode=%d, call=%d, damper=%s",
               zone, zone_data.ambient_temperature, zone_data.heating_setpoint, 
               zone_data.cooling_setpoint, zone_data.target_mode, zone_data.current_call,
               zone_data.damper_open ? "open" : "closed");
    }
  }
  
  // Compute and publish derived sensors (COP, delta-T, approach temperature)
  // These go beyond the Ruby gem — computed on-device from raw register values.
  this->publish_derived_sensors(regs);
  
  // Notify all registered listeners (sub-entities) that data has been updated.
  // This replaces the 5-second polling loop in each sub-entity, matching the
  // Ruby gem's "parent polls, pushes to children" pattern.
  for (auto &listener : this->listeners_) {
    listener();
  }
}

// REAL Control methods
bool WaterFurnaceAurora::set_heating_setpoint(float temp) {
  if (temp < 40.0f || temp > 90.0f) {
    ESP_LOGW(TAG, "Heating setpoint %.1f out of range (40-90)", temp);
    return false;
  }
  uint16_t value = static_cast<uint16_t>(temp * 10);
  return this->write_holding_register(registers::HEATING_SETPOINT_WRITE, value);
}

bool WaterFurnaceAurora::set_cooling_setpoint(float temp) {
  if (temp < 54.0f || temp > 99.0f) {
    ESP_LOGW(TAG, "Cooling setpoint %.1f out of range (54-99)", temp);
    return false;
  }
  uint16_t value = static_cast<uint16_t>(temp * 10);
  return this->write_holding_register(registers::COOLING_SETPOINT_WRITE, value);
}

bool WaterFurnaceAurora::set_hvac_mode(HeatingMode mode) {
  return this->write_holding_register(registers::HEATING_MODE_WRITE, static_cast<uint16_t>(mode));
}

bool WaterFurnaceAurora::set_fan_mode(FanMode mode) {
  return this->write_holding_register(registers::FAN_MODE_WRITE, static_cast<uint16_t>(mode));
}

bool WaterFurnaceAurora::set_dhw_enabled(bool enabled) {
  return this->write_holding_register(registers::DHW_ENABLED, enabled ? 1 : 0);
}

bool WaterFurnaceAurora::set_dhw_setpoint(float temp) {
  if (temp < 100.0f || temp > 140.0f) {
    ESP_LOGW(TAG, "DHW setpoint %.1f out of range (100-140)", temp);
    return false;
  }
  uint16_t value = static_cast<uint16_t>(temp * 10);
  return this->write_holding_register(registers::DHW_SETPOINT, value);
}

bool WaterFurnaceAurora::set_blower_only_speed(uint8_t speed) {
  if (speed < 1 || speed > 12) {
    ESP_LOGW(TAG, "Blower only speed %d out of range (1-12)", speed);
    return false;
  }
  return this->write_holding_register(registers::BLOWER_ONLY_SPEED, speed);
}

bool WaterFurnaceAurora::set_lo_compressor_speed(uint8_t speed) {
  if (speed < 1 || speed > 12) {
    ESP_LOGW(TAG, "Lo compressor speed %d out of range (1-12)", speed);
    return false;
  }
  return this->write_holding_register(registers::LO_COMPRESSOR_ECM_SPEED, speed);
}

bool WaterFurnaceAurora::set_hi_compressor_speed(uint8_t speed) {
  if (speed < 1 || speed > 12) {
    ESP_LOGW(TAG, "Hi compressor speed %d out of range (1-12)", speed);
    return false;
  }
  return this->write_holding_register(registers::HI_COMPRESSOR_ECM_SPEED, speed);
}

bool WaterFurnaceAurora::set_aux_heat_ecm_speed(uint8_t speed) {
  if (speed < 1 || speed > 12) {
    ESP_LOGW(TAG, "Aux heat ECM speed %d out of range (1-12)", speed);
    return false;
  }
  return this->write_holding_register(registers::AUX_HEAT_ECM_SPEED, speed);
}

bool WaterFurnaceAurora::set_pump_speed(uint8_t speed) {
  if (speed < 1 || speed > 100) {
    ESP_LOGW(TAG, "Pump speed %d out of range (1-100)", speed);
    return false;
  }
  return this->write_holding_register(registers::VS_PUMP_MANUAL, speed);
}

bool WaterFurnaceAurora::set_pump_min_speed(uint8_t speed) {
  if (speed < 1 || speed > 100) {
    ESP_LOGW(TAG, "Pump min speed %d out of range (1-100)", speed);
    return false;
  }
  return this->write_holding_register(registers::VS_PUMP_MIN, speed);
}

bool WaterFurnaceAurora::set_pump_max_speed(uint8_t speed) {
  if (speed < 1 || speed > 100) {
    ESP_LOGW(TAG, "Pump max speed %d out of range (1-100)", speed);
    return false;
  }
  return this->write_holding_register(registers::VS_PUMP_MAX, speed);
}

bool WaterFurnaceAurora::set_fan_intermittent_on(uint8_t minutes) {
  if (minutes > 25 || (minutes % 5) != 0) {
    ESP_LOGW(TAG, "Fan intermittent on time %d invalid (0, 5, 10, 15, 20, 25)", minutes);
    return false;
  }
  return this->write_holding_register(registers::FAN_INTERMITTENT_ON_WRITE, minutes);
}

bool WaterFurnaceAurora::set_fan_intermittent_off(uint8_t minutes) {
  if (minutes < 5 || minutes > 40 || (minutes % 5) != 0) {
    ESP_LOGW(TAG, "Fan intermittent off time %d invalid (5, 10, 15, 20, 25, 30, 35, 40)", minutes);
    return false;
  }
  return this->write_holding_register(registers::FAN_INTERMITTENT_OFF_WRITE, minutes);
}

bool WaterFurnaceAurora::set_humidification_target(uint8_t percent) {
  if (percent < 15 || percent > 50) {
    ESP_LOGW(TAG, "Humidification target %d out of range (15-50)", percent);
    return false;
  }
  // Read current targets to preserve the dehumidification value
  uint16_t read_reg = (this->has_iz2_ && this->awl_communicating())
                          ? registers::IZ2_HUMIDISTAT_TARGETS    // 31110
                          : registers::HUMIDISTAT_TARGETS;       // 12310
  uint16_t write_reg = (this->has_iz2_ && this->awl_communicating())
                           ? registers::IZ2_HUMIDISTAT_TARGETS_WRITE  // 21115
                           : registers::HUMIDISTAT_TARGETS;           // 12310
  std::vector<uint16_t> result;
  if (!this->read_holding_registers(read_reg, 1, result) || result.empty()) {
    ESP_LOGW(TAG, "Failed to read current humidistat targets");
    return false;
  }
  uint8_t dehum_target = result[0] & 0xFF;
  uint16_t value = (percent << 8) | dehum_target;
  return this->write_holding_register(write_reg, value);
}

bool WaterFurnaceAurora::set_dehumidification_target(uint8_t percent) {
  if (percent < 35 || percent > 65) {
    ESP_LOGW(TAG, "Dehumidification target %d out of range (35-65)", percent);
    return false;
  }
  // Read current targets to preserve the humidification value
  uint16_t read_reg = (this->has_iz2_ && this->awl_communicating())
                          ? registers::IZ2_HUMIDISTAT_TARGETS    // 31110
                          : registers::HUMIDISTAT_TARGETS;       // 12310
  uint16_t write_reg = (this->has_iz2_ && this->awl_communicating())
                           ? registers::IZ2_HUMIDISTAT_TARGETS_WRITE  // 21115
                           : registers::HUMIDISTAT_TARGETS;           // 12310
  std::vector<uint16_t> result;
  if (!this->read_holding_registers(read_reg, 1, result) || result.empty()) {
    ESP_LOGW(TAG, "Failed to read current humidistat targets");
    return false;
  }
  uint8_t hum_target = (result[0] >> 8) & 0xFF;
  uint16_t value = (hum_target << 8) | percent;
  return this->write_holding_register(write_reg, value);
}

bool WaterFurnaceAurora::clear_fault_history() {
  ESP_LOGI(TAG, "Clearing fault history");
  return this->write_holding_register(registers::CLEAR_FAULT_HISTORY, registers::CLEAR_FAULT_MAGIC);
}

// REAL IZ2 Zone controls
bool WaterFurnaceAurora::set_zone_heating_setpoint(uint8_t zone_number, float temp) {
  if (!this->validate_zone_number(zone_number)) return false;
  if (temp < 40.0f || temp > 90.0f) {
    ESP_LOGW(TAG, "Heating setpoint %.1f out of range (40-90)", temp);
    return false;
  }
  uint16_t reg = registers::IZ2_HEAT_SP_WRITE_BASE + ((zone_number - 1) * 9);
  uint16_t value = static_cast<uint16_t>(temp * 10);
  return this->write_holding_register(reg, value);
}

bool WaterFurnaceAurora::set_zone_cooling_setpoint(uint8_t zone_number, float temp) {
  if (!this->validate_zone_number(zone_number)) return false;
  if (temp < 54.0f || temp > 99.0f) {
    ESP_LOGW(TAG, "Cooling setpoint %.1f out of range (54-99)", temp);
    return false;
  }
  uint16_t reg = registers::IZ2_COOL_SP_WRITE_BASE + ((zone_number - 1) * 9);
  uint16_t value = static_cast<uint16_t>(temp * 10);
  return this->write_holding_register(reg, value);
}

bool WaterFurnaceAurora::set_zone_hvac_mode(uint8_t zone_number, HeatingMode mode) {
  if (!this->validate_zone_number(zone_number)) return false;
  uint16_t reg = registers::IZ2_MODE_WRITE_BASE + ((zone_number - 1) * 9);
  return this->write_holding_register(reg, static_cast<uint16_t>(mode));
}

bool WaterFurnaceAurora::set_zone_fan_mode(uint8_t zone_number, FanMode mode) {
  if (!this->validate_zone_number(zone_number)) return false;
  uint16_t reg = registers::IZ2_FAN_MODE_WRITE_BASE + ((zone_number - 1) * 9);
  return this->write_holding_register(reg, static_cast<uint16_t>(mode));
}

bool WaterFurnaceAurora::set_zone_fan_intermittent_on(uint8_t zone_number, uint8_t minutes) {
  if (!this->validate_zone_number(zone_number)) return false;
  if (minutes > 25 || (minutes % 5) != 0) {
    ESP_LOGW(TAG, "Fan intermittent on time %d invalid (0, 5, 10, 15, 20, 25)", minutes);
    return false;
  }
  uint16_t reg = registers::IZ2_FAN_ON_WRITE_BASE + ((zone_number - 1) * 9);
  return this->write_holding_register(reg, minutes);
}

bool WaterFurnaceAurora::set_zone_fan_intermittent_off(uint8_t zone_number, uint8_t minutes) {
  if (!this->validate_zone_number(zone_number)) return false;
  if (minutes < 5 || minutes > 40 || (minutes % 5) != 0) {
    ESP_LOGW(TAG, "Fan intermittent off time %d invalid (5, 10, 15, 20, 25, 30, 35, 40)", minutes);
    return false;
  }
  uint16_t reg = registers::IZ2_FAN_OFF_WRITE_BASE + ((zone_number - 1) * 9);
  return this->write_holding_register(reg, minutes);
}

// REAL Modbus communication functions

// WaterFurnace custom protocol: Function 0x42 ('B') - Read specific registers
bool WaterFurnaceAurora::read_specific_registers(const std::vector<uint16_t> &addresses,
                                                   RegisterMap &result) {
  result.clear();
  
  if (addresses.empty()) {
    return true;
  }
  
  // Pre-allocate request buffer to avoid stack growth
  // Size: 1 (addr) + 1 (func) + addresses*2 + 2 (CRC)
  std::vector<uint8_t> request;
  request.reserve(4 + addresses.size() * 2);
  
  // Pre-allocate response buffer
  std::vector<uint8_t> response;
  response.reserve(256);
  
  // Retry loop (like Ruby's read_retries: 2)
  for (uint8_t attempt = 0; attempt <= this->read_retries_; attempt++) {
    if (attempt > 0) {
      ESP_LOGD(TAG, "Retry %d for 0x42 read", attempt);
      delay(50);
      yield();  // Give RTOS time
    }
    
    // Build request: address + function + list of register addresses
    request.clear();
    request.push_back(this->address_);
    request.push_back(FUNC_READ_SPECIFIC);  // 0x42 = 'B'
    
    for (uint16_t addr : addresses) {
      request.push_back(addr >> 8);
      request.push_back(addr & 0xFF);
    }
    
    // Add CRC
    uint16_t crc = this->calculate_crc(request.data(), request.size());
    request.push_back(crc & 0xFF);
    request.push_back(crc >> 8);
    
    ESP_LOGV(TAG, "Sent 0x42 request for %d registers", addresses.size());
    
    // Send and wait for response - use shorter timeout (500ms) to fail faster when no hardware
    if (!this->send_and_receive(request.data(), request.size(), response, 500)) {
      yield();  // Give RTOS time before retry
      continue;
    }
    
    // Parse response
    if (response.size() < 5) {
      ESP_LOGW(TAG, "Response too short: %d bytes", response.size());
      continue;
    }
    
    if (response[0] != this->address_) {
      ESP_LOGW(TAG, "Wrong address in response: 0x%02X", response[0]);
      continue;
    }
    
    if (response[1] != FUNC_READ_SPECIFIC) {
      ESP_LOGW(TAG, "Wrong function in response: 0x%02X", response[1]);
      continue;
    }
    
    uint8_t byte_count = response[2];
    if (response.size() < static_cast<size_t>(3 + byte_count + 2)) {
      ESP_LOGW(TAG, "Response truncated: expected %d, got %d", 3 + byte_count + 2, response.size());
      continue;
    }
    
    // Parse register values — build sorted vector for O(log n) lookup
    size_t data_start = 3;
    size_t count = std::min(addresses.size(), static_cast<size_t>(byte_count / 2));
    result.reserve(count);
    for (size_t i = 0; i < count; i++) {
      uint16_t value = (response[data_start + i * 2] << 8) | response[data_start + i * 2 + 1];
      result.emplace_back(addresses[i], value);
    }
    // Sort by register address for binary search
    std::sort(result.begin(), result.end(),
        [](const std::pair<uint16_t, uint16_t> &a, const std::pair<uint16_t, uint16_t> &b) {
          return a.first < b.first;
        });
    
    ESP_LOGV(TAG, "Received %d register values", result.size());
    return true;
  }
  
  ESP_LOGD(TAG, "Failed to read registers after %d retries", this->read_retries_);
  return false;
}

// Function 0x41 ('A') - Read multiple register ranges
bool WaterFurnaceAurora::read_register_ranges(const std::vector<std::pair<uint16_t, uint16_t>> &ranges,
                                                RegisterMap &result) {
  result.clear();
  
  if (ranges.empty()) {
    return true;
  }
  
  // Pre-allocate request buffer: 1 (addr) + 1 (func) + ranges*4 + 2 (CRC)
  std::vector<uint8_t> request;
  request.reserve(4 + ranges.size() * 4);
  
  // Retry loop (matching read_specific_registers and Ruby gem's read_retries: 2)
  for (uint8_t attempt = 0; attempt <= this->read_retries_; attempt++) {
    if (attempt > 0) {
      ESP_LOGD(TAG, "Retry %d for 0x41 read", attempt);
      delay(50);
      yield();
    }
    
    // Build request
    request.clear();
    request.push_back(this->address_);
    request.push_back(FUNC_READ_RANGES);  // 0x41 = 'A'
    
    for (const auto &range : ranges) {
      request.push_back(range.first >> 8);
      request.push_back(range.first & 0xFF);
      request.push_back(range.second >> 8);
      request.push_back(range.second & 0xFF);
    }
    
    // Add CRC
    uint16_t crc = this->calculate_crc(request.data(), request.size());
    request.push_back(crc & 0xFF);
    request.push_back(crc >> 8);
    
    // Send request and wait for response
    std::vector<uint8_t> response;
    if (!this->send_and_receive(request.data(), request.size(), response, 2000)) {
      yield();
      continue;
    }
    
    // Parse response — validate address and function code
    if (response.size() < 5) {
      ESP_LOGW(TAG, "Response too short for 0x41 read: %d bytes", response.size());
      continue;
    }
    
    if (response[0] != this->address_) {
      ESP_LOGW(TAG, "Wrong address in 0x41 response: 0x%02X (expected 0x%02X)", response[0], this->address_);
      continue;
    }
    
    if (response[1] != FUNC_READ_RANGES) {
      ESP_LOGW(TAG, "Wrong function in 0x41 response: 0x%02X", response[1]);
      continue;
    }
    
    uint8_t byte_count = response[2];
    size_t data_start = 3;
    size_t data_idx = 0;
    
    // Map response data back to register addresses — ranges are already sequential
    for (const auto &range : ranges) {
      for (uint16_t i = 0; i < range.second && (data_idx * 2) < byte_count; i++) {
        uint16_t value = (response[data_start + data_idx * 2] << 8) | 
                         response[data_start + data_idx * 2 + 1];
        result.emplace_back(range.first + i, value);
        data_idx++;
      }
    }
    // Ranges produce sequential addresses — result is already sorted
    return true;
  }
  
  ESP_LOGD(TAG, "Failed to read register ranges after %d retries", this->read_retries_);
  return false;
}

// Standard Modbus function 0x03 - Read holding registers
bool WaterFurnaceAurora::read_holding_registers(uint16_t start_addr, uint16_t count, 
                                                  std::vector<uint16_t> &result) {
  result.clear();
  
  // Retry loop
  for (uint8_t attempt = 0; attempt <= this->read_retries_; attempt++) {
    if (attempt > 0) {
      ESP_LOGD(TAG, "Retry %d for 0x03 read at %d", attempt, start_addr);
      delay(50);
    }
    
    // Build request
    uint8_t request[8];
    request[0] = this->address_;
    request[1] = FUNC_READ_HOLDING;
    request[2] = start_addr >> 8;
    request[3] = start_addr & 0xFF;
    request[4] = count >> 8;
    request[5] = count & 0xFF;
    
    uint16_t crc = this->calculate_crc(request, 6);
    request[6] = crc & 0xFF;
    request[7] = crc >> 8;
    
    // Send request and wait for response
    std::vector<uint8_t> response;
    if (!this->send_and_receive(request, 8, response, 1000)) {
      continue;
    }
    
    // Parse response — validate address and function code (matches read_specific_registers)
    if (response.size() < 5) {
      ESP_LOGW(TAG, "Response too short for 0x03 read: %d bytes", response.size());
      continue;
    }
    
    if (response[0] != this->address_) {
      ESP_LOGW(TAG, "Wrong address in 0x03 response: 0x%02X (expected 0x%02X)", response[0], this->address_);
      continue;
    }
    
    if (response[1] != FUNC_READ_HOLDING) {
      ESP_LOGW(TAG, "Wrong function in 0x03 response: 0x%02X", response[1]);
      continue;
    }
    
    uint8_t byte_count = response[2];
    for (size_t i = 0; i < byte_count; i += 2) {
      uint16_t value = (response[3 + i] << 8) | response[3 + i + 1];
      result.push_back(value);
    }
    
    return true;
  }
  
  ESP_LOGW(TAG, "Failed to read holding registers at %d after %d retries", start_addr, this->read_retries_);
  return false;
}

// Standard Modbus function 0x06 - Write single register
bool WaterFurnaceAurora::write_holding_register(uint16_t addr, uint16_t value) {
  // Build request
  uint8_t request[8];
  request[0] = this->address_;
  request[1] = FUNC_WRITE_SINGLE;
  request[2] = addr >> 8;
  request[3] = addr & 0xFF;
  request[4] = value >> 8;
  request[5] = value & 0xFF;
  
  uint16_t crc = this->calculate_crc(request, 6);
  request[6] = crc & 0xFF;
  request[7] = crc >> 8;
  
  ESP_LOGD(TAG, "Writing register %d = %d", addr, value);
  
  // Send request and wait for response (echo of request)
  std::vector<uint8_t> response;
  return this->send_and_receive(request, 8, response, 1000);
}

// Shared Modbus transport: clear bus, TX, send, flush, delay, RX, wait for response
bool WaterFurnaceAurora::send_and_receive(const uint8_t *request, size_t request_len,
                                           std::vector<uint8_t> &response, uint32_t timeout_ms) {
  // Clear any pending data on the bus
  while (this->available()) {
    this->read();
  }
  
  // Enable TX mode for RS485
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(true);
  }
  
  // Send request
  this->write_array(request, request_len);
  this->flush();
  
  delayMicroseconds(500);
  
  // Disable TX mode (enable RX) for RS485
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(false);
  }
  
  // Wait for response
  response.clear();
  return this->wait_for_response(response, timeout_ms);
}

// Standard Modbus CRC-16 calculation
uint16_t WaterFurnaceAurora::calculate_crc(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

bool WaterFurnaceAurora::wait_for_response(std::vector<uint8_t> &response, uint32_t timeout_ms) {
  response.clear();
  response.reserve(256);  // Pre-allocate to avoid reallocation
  uint32_t start = millis();
  uint32_t yield_counter = 0;
  
  while (millis() - start < timeout_ms) {
    // Yield periodically to prevent watchdog timeout
    if (++yield_counter % 50 == 0) {
      yield();
    }
    
    while (this->available() && response.size() < 512) {
      response.push_back(this->read());
    }
    if (response.size() >= 512) {
      ESP_LOGW(TAG, "Response buffer overflow (>=512 bytes) - bus noise?");
      return false;
    }
    
    // Check if we have a complete response
    if (response.size() >= 5) {
      if (response[0] == this->address_) {
        uint8_t func = response[1];
        
        // Check for exception response (high bit set)
        if (func & 0x80) {
          ESP_LOGW(TAG, "Modbus exception: function 0x%02X, code 0x%02X", func, response[2]);
          return false;
        }
        
        // For read functions (0x03, 0x41, 0x42), check if complete
        if (func == FUNC_READ_HOLDING || func == FUNC_READ_RANGES || func == FUNC_READ_SPECIFIC) {
          uint8_t byte_count = response[2];
          size_t expected_len = 3 + byte_count + 2;
          if (response.size() >= expected_len) {
            // Verify CRC
            uint16_t received_crc = (response[expected_len - 1] << 8) | response[expected_len - 2];
            uint16_t calc_crc = this->calculate_crc(response.data(), expected_len - 2);
            if (received_crc == calc_crc) {
              return true;
            } else {
              ESP_LOGW(TAG, "CRC mismatch: received 0x%04X, calculated 0x%04X", received_crc, calc_crc);
              return false;
            }
          }
        }
        
        // For write single register (function 6), response is echo of request (8 bytes)
        if (func == FUNC_WRITE_SINGLE && response.size() >= 8) {
          uint16_t received_crc = (response[7] << 8) | response[6];
          uint16_t calc_crc = this->calculate_crc(response.data(), 6);
          if (received_crc == calc_crc) {
            return true;
          } else {
            ESP_LOGW(TAG, "Write CRC mismatch");
            return false;
          }
        }
      }
    }
    
    delay(1);
  }
  
  ESP_LOGD(TAG, "Modbus timeout waiting for response (got %d bytes)", response.size());
  return false;
}

// REAL Data conversion helpers
float WaterFurnaceAurora::to_signed_tenths(uint16_t value) {
  int16_t signed_val = static_cast<int16_t>(value);
  return signed_val / 10.0f;
}

float WaterFurnaceAurora::to_tenths(uint16_t value) {
  return value / 10.0f;
}

uint32_t WaterFurnaceAurora::to_uint32(uint16_t high, uint16_t low) {
  return (static_cast<uint32_t>(high) << 16) | low;
}

int32_t WaterFurnaceAurora::to_int32(uint16_t high, uint16_t low) {
  return static_cast<int32_t>(to_uint32(high, low));
}

std::string WaterFurnaceAurora::registers_to_string(const std::vector<uint16_t> &regs) {
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
  
  // Strip trailing spaces (matches Ruby gem behavior)
  while (!result.empty() && (result.back() == ' ' || result.back() == '\0')) {
    result.pop_back();
  }
  
  return result;
}

// String helpers - REAL IMPLEMENTATIONS
std::string WaterFurnaceAurora::get_vs_derate_string(uint16_t value) {
  if (value == 0) return "None";
  
  std::string result;
  result.reserve(64);
  if (value & VS_DERATE_DRIVE_OVER_TEMP) {
    if (!result.empty()) result += ", ";
    result += "Drive Over Temp";
  }
  if (value & VS_DERATE_LOW_SUCTION_PRESSURE) {
    if (!result.empty()) result += ", ";
    result += "Low Suction Pressure";
  }
  if (value & VS_DERATE_LOW_DISCHARGE_PRESSURE) {
    if (!result.empty()) result += ", ";
    result += "Low Discharge Pressure";
  }
  if (value & VS_DERATE_HIGH_DISCHARGE_PRESSURE) {
    if (!result.empty()) result += ", ";
    result += "High Discharge Pressure";
  }
  if (value & VS_DERATE_OUTPUT_POWER_LIMIT) {
    if (!result.empty()) result += ", ";
    result += "Output Power Limit";
  }
  
  return result.empty() ? "Unknown" : result;
}

std::string WaterFurnaceAurora::get_vs_safe_mode_string(uint16_t value) {
  if (value == 0) return "None";
  
  std::string result;
  result.reserve(64);
  if (value & VS_SAFE_EEV_INDOOR_FAILED) {
    if (!result.empty()) result += ", ";
    result += "EEV Indoor Failed";
  }
  if (value & VS_SAFE_EEV_OUTDOOR_FAILED) {
    if (!result.empty()) result += ", ";
    result += "EEV Outdoor Failed";
  }
  if (value & VS_SAFE_INVALID_AMBIENT_TEMP) {
    if (!result.empty()) result += ", ";
    result += "Invalid Ambient Temp";
  }
  
  return result.empty() ? "Unknown" : result;
}

std::string WaterFurnaceAurora::get_vs_alarm_string(uint16_t alarm1, uint16_t alarm2) {
  if (alarm1 == 0 && alarm2 == 0) return "None";
  
  std::string result;
  result.reserve(128);
  
  // Alarm1 flags
  if (alarm1 & 0x8000) {
    if (!result.empty()) result += ", ";
    result += "Internal Error";
  }
  
  // Alarm2 flags
  if (alarm2 & 0x0001) {
    if (!result.empty()) result += ", ";
    result += "Multi Safe Modes";
  }
  if (alarm2 & 0x0002) {
    if (!result.empty()) result += ", ";
    result += "Out of Envelope";
  }
  if (alarm2 & 0x0004) {
    if (!result.empty()) result += ", ";
    result += "Over Current";
  }
  if (alarm2 & 0x0008) {
    if (!result.empty()) result += ", ";
    result += "Over Voltage";
  }
  if (alarm2 & 0x0010) {
    if (!result.empty()) result += ", ";
    result += "Drive Over Temp";
  }
  if (alarm2 & 0x0020) {
    if (!result.empty()) result += ", ";
    result += "Under Voltage";
  }
  if (alarm2 & 0x0040) {
    if (!result.empty()) result += ", ";
    result += "High Discharge Temp";
  }
  if (alarm2 & 0x0080) {
    if (!result.empty()) result += ", ";
    result += "Invalid Discharge Temp";
  }
  if (alarm2 & 0x0100) {
    if (!result.empty()) result += ", ";
    result += "OEM Comms Timeout";
  }
  if (alarm2 & 0x0200) {
    if (!result.empty()) result += ", ";
    result += "MOC Safety";
  }
  if (alarm2 & 0x0400) {
    if (!result.empty()) result += ", ";
    result += "DC Under Voltage";
  }
  if (alarm2 & 0x0800) {
    if (!result.empty()) result += ", ";
    result += "Invalid Suction Pressure";
  }
  if (alarm2 & 0x1000) {
    if (!result.empty()) result += ", ";
    result += "Invalid Discharge Pressure";
  }
  if (alarm2 & 0x2000) {
    if (!result.empty()) result += ", ";
    result += "Low Discharge Pressure";
  }
  
  return result.empty() ? "Unknown" : result;
}

std::string WaterFurnaceAurora::get_axb_inputs_string(uint16_t value) {
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

// Derived sensors — computed on-device from raw register values.
// These go beyond the Ruby gem, leveraging our low-level access.
//
// COP = Heat of Rejection (or Extraction) / Total Watts
//   Heating COP = Heat of Rejection (BTU/h) / (Total Watts * 3.412 BTU/Wh)
//   Cooling COP = Heat of Extraction (BTU/h) / (Total Watts * 3.412 BTU/Wh)
// Only meaningful when compressor is running and power > 0.
//
// Water Delta-T = Leaving Water - Entering Water
//   Positive in heating mode (heat added to water), negative in cooling.
//
// Approach Temperature = how close the heat exchanger approaches ideal
//   Heating: Leaving Water - Saturated Condenser Temp
//   Cooling: Saturated Condenser Temp - Entering Water
constexpr float BTU_PER_WATT_HOUR = 3.412f;

void WaterFurnaceAurora::publish_derived_sensors(const RegisterMap &regs) {
  // Water delta-T (requires AXB with leaving/entering water temps)
  if (this->water_delta_t_sensor_ != nullptr) {
    const uint16_t *val_lw = reg_find(regs, registers::LEAVING_WATER);
    const uint16_t *val_ew = reg_find(regs, registers::ENTERING_WATER);
    if (val_lw && val_ew) {
      float leaving = to_signed_tenths(*val_lw);
      float entering = to_signed_tenths(*val_ew);
      this->water_delta_t_sensor_->publish_state(leaving - entering);
    }
  }
  
  // COP — only valid when compressor is running and we have non-zero power
  if (this->cop_sensor_ != nullptr) {
    bool compressor_running = (this->system_outputs_ & (OUTPUT_CC | OUTPUT_CC2)) != 0;
    if (compressor_running) {
      const uint16_t *tw_h = reg_find(regs, registers::TOTAL_WATTS);
      const uint16_t *tw_l = reg_find(regs, registers::TOTAL_WATTS + 1);
      if (tw_h && tw_l) {
        uint32_t total_watts = to_uint32(*tw_h, *tw_l);
        if (total_watts > 0) {
          bool is_cooling = (this->system_outputs_ & OUTPUT_RV) != 0;
          // Select heat of extraction (cooling) or rejection (heating)
          uint16_t heat_reg = is_cooling ? registers::HEAT_OF_EXTRACTION : registers::HEAT_OF_REJECTION;
          const uint16_t *heat_h = reg_find(regs, heat_reg);
          const uint16_t *heat_l = reg_find(regs, heat_reg + 1);
          if (heat_h && heat_l) {
            int32_t heat_btu = to_int32(*heat_h, *heat_l);
            // COP = BTU/h output / BTU/h input; input = watts * 3.412 BTU/Wh
            float cop = static_cast<float>(std::abs(heat_btu)) / (static_cast<float>(total_watts) * BTU_PER_WATT_HOUR);
            // Sanity check: COP should be 1-10 for geothermal; reject outliers
            if (cop >= 0.5f && cop <= 15.0f) {
              this->cop_sensor_->publish_state(cop);
            }
          }
        }
      }
    } else {
      // Compressor off — publish 0 COP
      this->cop_sensor_->publish_state(0.0f);
    }
  }
  
  // Approach temperature (requires refrigeration monitoring)
  if (this->approach_temp_sensor_ != nullptr) {
    const uint16_t *val_lw = reg_find(regs, registers::LEAVING_WATER);
    const uint16_t *val_ew = reg_find(regs, registers::ENTERING_WATER);
    const uint16_t *val_sc = reg_find(regs, registers::SATURATED_CONDENSER_TEMP);
    if (val_sc) {
      float sat_cond = to_signed_tenths(*val_sc);
      bool is_cooling = (this->system_outputs_ & OUTPUT_RV) != 0;
      if (is_cooling && val_ew) {
        // Cooling: approach = saturated condenser - entering water
        float entering = to_signed_tenths(*val_ew);
        this->approach_temp_sensor_->publish_state(sat_cond - entering);
      } else if (!is_cooling && val_lw) {
        // Heating: approach = leaving water - saturated condenser
        float leaving = to_signed_tenths(*val_lw);
        this->approach_temp_sensor_->publish_state(leaving - sat_cond);
      }
    }
  }
}

// REAL read_fault_history
void WaterFurnaceAurora::read_fault_history() {
  if (this->fault_history_sensor_ == nullptr) {
    return;
  }
  
  std::vector<uint16_t> result;
  result.reserve(99);
  if (!this->read_holding_registers(registers::FAULT_HISTORY_START, 99, result)) {
    ESP_LOGW(TAG, "Failed to read fault history");
    return;
  }
  
  std::string history;
  history.reserve(256);
  int fault_count = 0;
  const int max_faults = 10;
  
  for (size_t i = 0; i < result.size() && fault_count < max_faults; i++) {
    uint16_t fault_value = result[i];
    if (fault_value == 0 || fault_value == 0xFFFF) continue;
    
    uint8_t fault_code = fault_value % 100;
    if (fault_code == 0) continue;
    
    if (!history.empty()) history += "; ";
    history += "E";
    history += std::to_string(fault_code);
    
    const char* desc = get_fault_description(fault_code);
    if (desc && strcmp(desc, "Unknown Fault") != 0) {
      history += " (";
      history += desc;
      history += ")";
    }
    
    fault_count++;
  }
  
  if (history.empty()) {
    history = "No faults";
  } else if (fault_count >= max_faults) {
    history += "...";
  }
  
  this->fault_history_sensor_->publish_state(history);
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
