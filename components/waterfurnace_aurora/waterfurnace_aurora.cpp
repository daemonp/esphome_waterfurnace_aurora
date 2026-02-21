#include "waterfurnace_aurora.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include <cstring>

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "waterfurnace_aurora";

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
  
  // Publish pump type (detected once during setup, won't change at runtime)
  if (this->pump_type_sensor_ != nullptr) {
    this->pump_type_sensor_->publish_state(get_pump_type_string(this->pump_type_));
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
  ESP_LOGI(TAG, "  Pump: %s", get_pump_type_string(this->pump_type_));
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
  
  // Read IZ2 zone count (register 483)
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
  
  // Detect pump type (register 413)
  const uint16_t *val_pump = reg_find(result, registers::PUMP_TYPE);
  if (val_pump) {
    uint16_t pval = *val_pump;
    if (pval <= 7) {
      this->pump_type_ = static_cast<PumpType>(pval);
    } else {
      this->pump_type_ = PumpType::OTHER;
    }
    ESP_LOGD(TAG, "Pump type register 413 = %d -> %s", pval, get_pump_type_string(this->pump_type_));
  }
  
  // Detect VS Drive via ABC Program (register 88, 4 registers = 8 chars ASCII)
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
      this->has_vs_drive_ = (program.find("VSP") != std::string::npos || 
                              program.find("SPLVS") != std::string::npos);
      ESP_LOGD(TAG, "VS Drive detection from program '%s' -> %s", 
               program.c_str(), this->has_vs_drive_ ? "present" : "absent");
    }
    
    // Fallback: probe VS drive-specific registers
    if (!this->has_vs_drive_) {
      std::vector<uint16_t> vs_probe = {3001, 3322, 3325};
      RegisterMap vs_result;
      if (this->read_specific_registers(vs_probe, vs_result)) {
        const uint16_t *v_speed = reg_find(vs_result, 3001);
        const uint16_t *v_press = reg_find(vs_result, 3322);
        const uint16_t *v_temp = reg_find(vs_result, 3325);
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
  ESP_LOGCONFIG(TAG, "  Pump: %s", get_pump_type_string(this->pump_type_));
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

// Adaptive polling tiers to reduce RS-485 bus contention:
//   Tier 0 (every cycle, ~5s): System outputs, temps, speeds, watts, pressures
//   Tier 1 (every 6th cycle, ~30s): Setpoints, mode config, humidistat, DHW settings
//   Tier 2 (every 60th cycle, ~5min): Fault history
void WaterFurnaceAurora::refresh_all_data() {
  bool medium_poll = (this->poll_tier_counter_ % 6) == 0;
  bool slow_poll = (this->poll_tier_counter_ % 60) == 0;
  
  this->addresses_to_read_.clear();
  if (this->addresses_to_read_.capacity() < 100) {
    this->addresses_to_read_.reserve(100);
  }
  
  // === TIER 0: Fast registers — every cycle (~5s) ===
  
  // Core system state
  this->addresses_to_read_.push_back(registers::COMPRESSOR_ANTI_SHORT_CYCLE);
  this->addresses_to_read_.push_back(registers::LAST_FAULT);
  this->addresses_to_read_.push_back(registers::SYSTEM_OUTPUTS);
  this->addresses_to_read_.push_back(registers::SYSTEM_STATUS);
  
  // Temperatures
  this->addresses_to_read_.push_back(registers::FP1_TEMP);
  this->addresses_to_read_.push_back(registers::FP2_TEMP);
  this->addresses_to_read_.push_back(registers::AMBIENT_TEMP);
  
  if (this->awl_axb()) {
    this->addresses_to_read_.push_back(registers::ENTERING_AIR_AWL);
    this->addresses_to_read_.push_back(registers::LEAVING_AIR);
  } else {
    this->addresses_to_read_.push_back(registers::ENTERING_AIR);
  }
  if (this->awl_communicating()) {
    this->addresses_to_read_.push_back(registers::RELATIVE_HUMIDITY);
    this->addresses_to_read_.push_back(registers::OUTDOOR_TEMP);
  }
  if (this->has_axb_) {
    this->addresses_to_read_.push_back(registers::AXB_OUTPUTS);
    this->addresses_to_read_.push_back(registers::LEAVING_WATER);
    this->addresses_to_read_.push_back(registers::ENTERING_WATER);
    this->addresses_to_read_.push_back(registers::WATERFLOW);
  }
  
  // Refrigeration monitoring
  if (this->refrigeration_monitoring()) {
    this->addresses_to_read_.push_back(registers::HEATING_LIQUID_LINE_TEMP);
    this->addresses_to_read_.push_back(registers::SATURATED_CONDENSER_TEMP);
    this->addresses_to_read_.push_back(registers::SUBCOOL_HEATING);
    this->addresses_to_read_.push_back(registers::SUBCOOL_COOLING);
    this->addresses_to_read_.push_back(registers::HEAT_OF_EXTRACTION);
    this->addresses_to_read_.push_back(registers::HEAT_OF_EXTRACTION + 1);
    this->addresses_to_read_.push_back(registers::HEAT_OF_REJECTION);
    this->addresses_to_read_.push_back(registers::HEAT_OF_REJECTION + 1);
  }
  
  // Energy monitoring
  if (this->energy_monitoring()) {
    this->addresses_to_read_.push_back(registers::LINE_VOLTAGE);
    this->addresses_to_read_.push_back(registers::COMPRESSOR_WATTS);
    this->addresses_to_read_.push_back(registers::COMPRESSOR_WATTS + 1);
    this->addresses_to_read_.push_back(registers::BLOWER_WATTS);
    this->addresses_to_read_.push_back(registers::BLOWER_WATTS + 1);
    this->addresses_to_read_.push_back(registers::AUX_WATTS);
    this->addresses_to_read_.push_back(registers::AUX_WATTS + 1);
    this->addresses_to_read_.push_back(registers::TOTAL_WATTS);
    this->addresses_to_read_.push_back(registers::TOTAL_WATTS + 1);
    this->addresses_to_read_.push_back(registers::PUMP_WATTS);
    this->addresses_to_read_.push_back(registers::PUMP_WATTS + 1);
  }
  
  // Blower speed
  if (this->is_ecm_blower()) {
    this->addresses_to_read_.push_back(registers::ECM_SPEED);
  } else if (this->blower_type_ == BlowerType::FIVE_SPEED) {
    this->addresses_to_read_.push_back(registers::ECM_SPEED);
  }
  
  // VS Drive real-time telemetry
  if (this->has_vs_drive_) {
    this->addresses_to_read_.push_back(registers::ACTIVE_DEHUMIDIFY);
    this->addresses_to_read_.push_back(registers::COMPRESSOR_SPEED_DESIRED);
    this->addresses_to_read_.push_back(registers::COMPRESSOR_SPEED_ACTUAL);
    this->addresses_to_read_.push_back(registers::VS_DISCHARGE_PRESSURE);
    this->addresses_to_read_.push_back(registers::VS_SUCTION_PRESSURE);
    this->addresses_to_read_.push_back(registers::VS_DISCHARGE_TEMP);
    this->addresses_to_read_.push_back(registers::VS_AMBIENT_TEMP);
    this->addresses_to_read_.push_back(registers::VS_DRIVE_TEMP);
    this->addresses_to_read_.push_back(registers::VS_INVERTER_TEMP);
    this->addresses_to_read_.push_back(registers::VS_COMPRESSOR_WATTS);
    this->addresses_to_read_.push_back(registers::VS_COMPRESSOR_WATTS + 1);
    this->addresses_to_read_.push_back(registers::VS_FAN_SPEED);
    this->addresses_to_read_.push_back(registers::VS_EEV_OPEN);
    this->addresses_to_read_.push_back(registers::VS_SUCTION_TEMP);
    this->addresses_to_read_.push_back(registers::VS_SAT_EVAP_DISCHARGE_TEMP);
    this->addresses_to_read_.push_back(registers::VS_SUPERHEAT_TEMP);
  }
  
  // VS pump speed
  if (this->has_axb_ && this->awl_axb()) {
    this->addresses_to_read_.push_back(registers::VS_PUMP_SPEED);
  }
  
  // IZ2 Zone data
  if (this->has_iz2_ && this->num_iz2_zones_ > 0) {
    this->addresses_to_read_.push_back(registers::IZ2_OUTDOOR_TEMP);
    this->addresses_to_read_.push_back(registers::IZ2_DEMAND);
    this->addresses_to_read_.push_back(registers::IZ2_COMPRESSOR_SPEED_DESIRED);
    this->addresses_to_read_.push_back(registers::IZ2_BLOWER_SPEED_DESIRED);
    
    for (uint8_t zone = 1; zone <= this->num_iz2_zones_; zone++) {
      this->addresses_to_read_.push_back(registers::IZ2_AMBIENT_BASE + ((zone - 1) * 3));
      this->addresses_to_read_.push_back(registers::IZ2_CONFIG1_BASE + ((zone - 1) * 3));
      this->addresses_to_read_.push_back(registers::IZ2_CONFIG2_BASE + ((zone - 1) * 3));
      this->addresses_to_read_.push_back(registers::IZ2_CONFIG3_BASE + ((zone - 1) * 3));
    }
  }
  
  // === TIER 1: Medium registers — every 6th cycle (~30s) ===
  if (medium_poll) {
    ESP_LOGD(TAG, "Medium poll cycle — reading setpoints, config, humidistat");
    
    this->addresses_to_read_.push_back(registers::LINE_VOLTAGE_SETTING);
    this->addresses_to_read_.push_back(registers::HEATING_SETPOINT);
    this->addresses_to_read_.push_back(registers::COOLING_SETPOINT);
    this->addresses_to_read_.push_back(registers::FAN_CONFIG);
    this->addresses_to_read_.push_back(registers::HEATING_MODE_READ);
    
    if (this->has_axb_) {
      this->addresses_to_read_.push_back(registers::DHW_ENABLED);
      this->addresses_to_read_.push_back(registers::DHW_SETPOINT);
      this->addresses_to_read_.push_back(registers::DHW_TEMP);
      this->addresses_to_read_.push_back(registers::LOOP_PRESSURE);
      this->addresses_to_read_.push_back(registers::AXB_INPUTS);
    }
    
    if (this->is_ecm_blower()) {
      this->addresses_to_read_.push_back(registers::BLOWER_ONLY_SPEED);
      this->addresses_to_read_.push_back(registers::LO_COMPRESSOR_ECM_SPEED);
      this->addresses_to_read_.push_back(registers::HI_COMPRESSOR_ECM_SPEED);
      this->addresses_to_read_.push_back(registers::AUX_HEAT_ECM_SPEED);
    }
    
    if (this->has_axb_) {
      this->addresses_to_read_.push_back(registers::VS_PUMP_MIN);
      this->addresses_to_read_.push_back(registers::VS_PUMP_MAX);
    }
    
    if (this->has_vs_drive_) {
      this->addresses_to_read_.push_back(registers::VS_DERATE);
      this->addresses_to_read_.push_back(registers::VS_SAFE_MODE);
      this->addresses_to_read_.push_back(registers::VS_ALARM1);
      this->addresses_to_read_.push_back(registers::VS_ALARM2);
    }
    
    if (this->has_iz2_ && this->awl_communicating()) {
      this->addresses_to_read_.push_back(registers::IZ2_HUMIDISTAT_SETTINGS);
      this->addresses_to_read_.push_back(registers::IZ2_HUMIDISTAT_MODE);
      this->addresses_to_read_.push_back(registers::IZ2_HUMIDISTAT_TARGETS);
    } else {
      this->addresses_to_read_.push_back(registers::HUMIDISTAT_SETTINGS);
      this->addresses_to_read_.push_back(registers::HUMIDISTAT_TARGETS);
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
  
  // Store in cache
  this->register_cache_ = std::move(result);
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
  
  // Temperatures
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
  
  // Setpoints
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
  
  // Power
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
  
  this->publish_sensor(regs, registers::COMPRESSOR_SPEED_DESIRED, this->compressor_desired_speed_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_DISCHARGE_TEMP, this->discharge_temp_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_SUCTION_TEMP, this->suction_temp_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_DRIVE_TEMP, this->vs_drive_temp_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_INVERTER_TEMP, this->vs_inverter_temp_sensor_);
  this->publish_sensor(regs, registers::VS_FAN_SPEED, this->vs_fan_speed_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_AMBIENT_TEMP, this->vs_ambient_temp_sensor_);
  this->publish_sensor_uint32(regs, registers::VS_COMPRESSOR_WATTS, this->vs_compressor_watts_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::VS_SAT_EVAP_DISCHARGE_TEMP, this->sat_evap_discharge_temp_sensor_);
  
  // VS Drive derate status
  {
    const uint16_t *val = reg_find(regs, registers::VS_DERATE);
    if (val) {
      this->publish_text_if_changed(this->vs_derate_sensor_, this->cached_vs_derate_,
                                     get_vs_derate_string(*val));
    }
  }
  
  // VS Drive safe mode status
  {
    const uint16_t *val = reg_find(regs, registers::VS_SAFE_MODE);
    if (val) {
      this->publish_text_if_changed(this->vs_safe_mode_sensor_, this->cached_vs_safe_mode_,
                                     get_vs_safe_mode_string(*val));
    }
  }
  
  // VS Drive alarm status
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
  
  // Heat of extraction/rejection
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
  
  // Humidistat
  {
    uint16_t mode_reg = (this->has_iz2_ && this->awl_communicating())
                            ? registers::IZ2_HUMIDISTAT_MODE
                            : registers::HUMIDISTAT_SETTINGS;
    uint16_t target_reg = (this->has_iz2_ && this->awl_communicating())
                              ? registers::IZ2_HUMIDISTAT_TARGETS
                              : registers::HUMIDISTAT_TARGETS;

    const uint16_t *val_mode = reg_find(regs, mode_reg);
    if (val_mode) {
      this->publish_text_if_changed(this->humidifier_mode_sensor_, this->cached_humidifier_mode_,
                                     (*val_mode & 0x8000) ? "Auto" : "Manual");
      this->publish_text_if_changed(this->dehumidifier_mode_sensor_, this->cached_dehumidifier_mode_,
                                     (*val_mode & 0x4000) ? "Auto" : "Manual");
    }

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
  
  // Parse IZ2 zone data — now uses iz2_extract_* helpers from registers.h
  if (this->has_iz2_ && this->num_iz2_zones_ > 0) {
    for (uint8_t zone = 1; zone <= this->num_iz2_zones_; zone++) {
      IZ2ZoneData& zone_data = this->iz2_zones_[zone - 1];
      
      const uint16_t *val_amb = reg_find(regs, registers::IZ2_AMBIENT_BASE + ((zone - 1) * 3));
      if (val_amb) {
        zone_data.ambient_temperature = to_signed_tenths(*val_amb);
      }
      
      const uint16_t *val_c1 = reg_find(regs, registers::IZ2_CONFIG1_BASE + ((zone - 1) * 3));
      if (val_c1) {
        uint16_t config1 = *val_c1;
        zone_data.target_fan_mode = iz2_extract_fan_mode(config1);
        zone_data.fan_on_time = iz2_extract_fan_on_time(config1);
        zone_data.fan_off_time = iz2_extract_fan_off_time(config1);
        zone_data.cooling_setpoint = iz2_extract_cooling_setpoint(config1);
      }
      
      const uint16_t *val_c2 = reg_find(regs, registers::IZ2_CONFIG2_BASE + ((zone - 1) * 3));
      if (val_c2 && val_c1) {
        uint16_t config2 = *val_c2;
        uint16_t config1 = *val_c1;
        zone_data.current_call = iz2_extract_current_call(config2);
        zone_data.target_mode = iz2_extract_mode(config2);
        zone_data.damper_open = iz2_extract_damper_open(config2);
        zone_data.heating_setpoint = iz2_extract_heating_setpoint(config1, config2);
      }
      
      const uint16_t *val_c3 = reg_find(regs, registers::IZ2_CONFIG3_BASE + ((zone - 1) * 3));
      if (val_c3) {
        uint16_t config3 = *val_c3;
        zone_data.priority = iz2_extract_priority(config3);
        zone_data.size = iz2_extract_size(config3);
        zone_data.normalized_size = iz2_extract_normalized_size(config3);
      }
      
      ESP_LOGV(TAG, "Zone %d: temp=%.1f, heat_sp=%.1f, cool_sp=%.1f, mode=%d, call=%d, damper=%s",
               zone, zone_data.ambient_temperature, zone_data.heating_setpoint, 
               zone_data.cooling_setpoint, zone_data.target_mode, zone_data.current_call,
               zone_data.damper_open ? "open" : "closed");
    }
    
    // IZ2 desired speeds
    this->publish_sensor(regs, registers::IZ2_COMPRESSOR_SPEED_DESIRED, this->iz2_compressor_speed_sensor_);
    
    {
      const uint16_t *val = reg_find(regs, registers::IZ2_BLOWER_SPEED_DESIRED);
      if (val && this->iz2_blower_speed_sensor_ != nullptr) {
        this->iz2_blower_speed_sensor_->publish_state(iz2_fan_desired(*val));
      }
    }
  }
  
  // Derived sensors (COP, delta-T, approach temperature)
  this->publish_derived_sensors(regs);
  
  // Notify all registered listeners
  for (auto &listener : this->listeners_) {
    listener();
  }
}

// Control methods
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
  uint16_t read_reg = (this->has_iz2_ && this->awl_communicating())
                          ? registers::IZ2_HUMIDISTAT_TARGETS
                          : registers::HUMIDISTAT_TARGETS;
  uint16_t write_reg = (this->has_iz2_ && this->awl_communicating())
                           ? registers::IZ2_HUMIDISTAT_TARGETS_WRITE
                           : registers::HUMIDISTAT_TARGETS;
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
  uint16_t read_reg = (this->has_iz2_ && this->awl_communicating())
                          ? registers::IZ2_HUMIDISTAT_TARGETS
                          : registers::HUMIDISTAT_TARGETS;
  uint16_t write_reg = (this->has_iz2_ && this->awl_communicating())
                           ? registers::IZ2_HUMIDISTAT_TARGETS_WRITE
                           : registers::HUMIDISTAT_TARGETS;
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

// IZ2 Zone controls
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

// Modbus communication — uses protocol module for CRC and frame building.
// Still blocking for now; async state machine is Phase 3.

bool WaterFurnaceAurora::read_specific_registers(const std::vector<uint16_t> &addresses,
                                                   RegisterMap &result) {
  result.clear();
  if (addresses.empty()) return true;
  
  // Build frame using protocol module
  auto request = protocol::build_read_registers_request(this->address_, addresses);
  if (request.empty()) {
    ESP_LOGW(TAG, "Failed to build 0x42 request (too many registers?)");
    return false;
  }
  
  std::vector<uint8_t> response;
  response.reserve(256);
  
  for (uint8_t attempt = 0; attempt <= this->read_retries_; attempt++) {
    if (attempt > 0) {
      ESP_LOGD(TAG, "Retry %d for 0x42 read", attempt);
      delay(50);
      yield();
    }
    
    ESP_LOGV(TAG, "Sent 0x42 request for %d registers", addresses.size());
    
    if (!this->send_and_receive(request.data(), request.size(), response, 500)) {
      yield();
      continue;
    }
    
    // Parse response using protocol module
    auto parsed = protocol::parse_frame(response.data(), response.size(), addresses);
    if (parsed.is_error) {
      ESP_LOGW(TAG, "0x42 response error (code 0x%02X)", parsed.error_code);
      continue;
    }
    
    // Convert ParsedResponse to RegisterMap (sorted)
    result.reserve(parsed.registers.size());
    for (const auto &rv : parsed.registers) {
      result.emplace_back(rv.address, rv.value);
    }
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

bool WaterFurnaceAurora::read_register_ranges(const std::vector<std::pair<uint16_t, uint16_t>> &ranges,
                                                RegisterMap &result) {
  result.clear();
  if (ranges.empty()) return true;
  
  auto request = protocol::build_read_ranges_request(this->address_, ranges);
  if (request.empty()) return false;
  
  for (uint8_t attempt = 0; attempt <= this->read_retries_; attempt++) {
    if (attempt > 0) {
      ESP_LOGD(TAG, "Retry %d for 0x41 read", attempt);
      delay(50);
      yield();
    }
    
    std::vector<uint8_t> response;
    if (!this->send_and_receive(request.data(), request.size(), response, 2000)) {
      yield();
      continue;
    }
    
    // Build expected addresses from ranges for parse_frame
    std::vector<uint16_t> expected_addrs;
    for (const auto &range : ranges) {
      for (uint16_t i = 0; i < range.second; i++) {
        expected_addrs.push_back(range.first + i);
      }
    }
    
    auto parsed = protocol::parse_frame(response.data(), response.size(), expected_addrs);
    if (parsed.is_error) {
      ESP_LOGW(TAG, "0x41 response error (code 0x%02X)", parsed.error_code);
      continue;
    }
    
    result.reserve(parsed.registers.size());
    for (const auto &rv : parsed.registers) {
      result.emplace_back(rv.address, rv.value);
    }
    // Ranges produce sequential addresses — result is already sorted
    return true;
  }
  
  ESP_LOGD(TAG, "Failed to read register ranges after %d retries", this->read_retries_);
  return false;
}

bool WaterFurnaceAurora::read_holding_registers(uint16_t start_addr, uint16_t count, 
                                                  std::vector<uint16_t> &result) {
  result.clear();
  
  auto request = protocol::build_read_holding_request(this->address_, start_addr, count);
  
  for (uint8_t attempt = 0; attempt <= this->read_retries_; attempt++) {
    if (attempt > 0) {
      ESP_LOGD(TAG, "Retry %d for 0x03 read at %d", attempt, start_addr);
      delay(50);
    }
    
    std::vector<uint8_t> response;
    if (!this->send_and_receive(request.data(), request.size(), response, 1000)) {
      continue;
    }
    
    // Build expected addresses
    std::vector<uint16_t> expected_addrs;
    expected_addrs.reserve(count);
    for (uint16_t i = 0; i < count; i++) {
      expected_addrs.push_back(start_addr + i);
    }
    
    auto parsed = protocol::parse_frame(response.data(), response.size(), expected_addrs);
    if (parsed.is_error) {
      ESP_LOGW(TAG, "0x03 response error at %d (code 0x%02X)", start_addr, parsed.error_code);
      continue;
    }
    
    result.reserve(parsed.registers.size());
    for (const auto &rv : parsed.registers) {
      result.push_back(rv.value);
    }
    
    return true;
  }
  
  ESP_LOGW(TAG, "Failed to read holding registers at %d after %d retries", start_addr, this->read_retries_);
  return false;
}

bool WaterFurnaceAurora::write_holding_register(uint16_t addr, uint16_t value) {
  auto request = protocol::build_write_single_request(this->address_, addr, value);
  
  ESP_LOGD(TAG, "Writing register %d = %d", addr, value);
  
  std::vector<uint8_t> response;
  return this->send_and_receive(request.data(), request.size(), response, 1000);
}

// Shared Modbus transport
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

bool WaterFurnaceAurora::wait_for_response(std::vector<uint8_t> &response, uint32_t timeout_ms) {
  response.clear();
  response.reserve(256);
  uint32_t start = millis();
  uint32_t yield_counter = 0;
  
  while (millis() - start < timeout_ms) {
    if (++yield_counter % 50 == 0) {
      yield();
    }
    
    while (this->available() && response.size() < protocol::MAX_FRAME_SIZE) {
      response.push_back(this->read());
    }
    if (response.size() >= protocol::MAX_FRAME_SIZE) {
      ESP_LOGW(TAG, "Response buffer overflow (>=%d bytes) - bus noise?", protocol::MAX_FRAME_SIZE);
      return false;
    }
    
    // Use protocol module to determine expected frame size
    if (response.size() >= 2) {
      size_t expected = protocol::expected_frame_size(response.data(), response.size());
      if (expected > 0 && response.size() >= expected) {
        // Frame is complete — validate CRC
        if (protocol::validate_frame_crc(response.data(), expected)) {
          return true;
        } else {
          ESP_LOGW(TAG, "CRC mismatch in response");
          return false;
        }
      }
    }
    
    delay(1);
  }
  
  ESP_LOGD(TAG, "Modbus timeout waiting for response (got %d bytes)", response.size());
  return false;
}

// Derived sensors
constexpr float BTU_PER_WATT_HOUR = 3.412f;

void WaterFurnaceAurora::publish_derived_sensors(const RegisterMap &regs) {
  // Water delta-T
  if (this->water_delta_t_sensor_ != nullptr) {
    const uint16_t *val_lw = reg_find(regs, registers::LEAVING_WATER);
    const uint16_t *val_ew = reg_find(regs, registers::ENTERING_WATER);
    if (val_lw && val_ew) {
      float leaving = to_signed_tenths(*val_lw);
      float entering = to_signed_tenths(*val_ew);
      this->water_delta_t_sensor_->publish_state(leaving - entering);
    }
  }
  
  // COP
  if (this->cop_sensor_ != nullptr) {
    bool compressor_running = (this->system_outputs_ & (OUTPUT_CC | OUTPUT_CC2)) != 0;
    if (compressor_running) {
      const uint16_t *tw_h = reg_find(regs, registers::TOTAL_WATTS);
      const uint16_t *tw_l = reg_find(regs, registers::TOTAL_WATTS + 1);
      if (tw_h && tw_l) {
        uint32_t total_watts = to_uint32(*tw_h, *tw_l);
        if (total_watts > 0) {
          bool is_cooling = (this->system_outputs_ & OUTPUT_RV) != 0;
          uint16_t heat_reg = is_cooling ? registers::HEAT_OF_EXTRACTION : registers::HEAT_OF_REJECTION;
          const uint16_t *heat_h = reg_find(regs, heat_reg);
          const uint16_t *heat_l = reg_find(regs, heat_reg + 1);
          if (heat_h && heat_l) {
            int32_t heat_btu = to_int32(*heat_h, *heat_l);
            float cop = static_cast<float>(std::abs(heat_btu)) / (static_cast<float>(total_watts) * BTU_PER_WATT_HOUR);
            if (cop >= 0.5f && cop <= 15.0f) {
              this->cop_sensor_->publish_state(cop);
            }
          }
        }
      }
    } else {
      this->cop_sensor_->publish_state(0.0f);
    }
  }
  
  // Approach temperature
  if (this->approach_temp_sensor_ != nullptr) {
    const uint16_t *val_lw = reg_find(regs, registers::LEAVING_WATER);
    const uint16_t *val_ew = reg_find(regs, registers::ENTERING_WATER);
    const uint16_t *val_sc = reg_find(regs, registers::SATURATED_CONDENSER_TEMP);
    if (val_sc) {
      float sat_cond = to_signed_tenths(*val_sc);
      bool is_cooling = (this->system_outputs_ & OUTPUT_RV) != 0;
      if (is_cooling && val_ew) {
        float entering = to_signed_tenths(*val_ew);
        this->approach_temp_sensor_->publish_state(sat_cond - entering);
      } else if (!is_cooling && val_lw) {
        float leaving = to_signed_tenths(*val_lw);
        this->approach_temp_sensor_->publish_state(leaving - sat_cond);
      }
    }
  }
}

// Fault history
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
