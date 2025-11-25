#include "waterfurnace_aurora.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "waterfurnace_aurora";

// WaterFurnace custom function codes (from lib/aurora/modbus/slave.rb)
static const uint8_t FUNC_READ_RANGES = 0x41;      // 'A' - read multiple ranges
static const uint8_t FUNC_READ_SPECIFIC = 0x42;   // 'B' - read specific addresses
static const uint8_t FUNC_READ_HOLDING = 0x03;    // Standard read holding registers
static const uint8_t FUNC_WRITE_SINGLE = 0x06;    // Standard write single register

// Fault descriptions from registers.rb FAULTS hash
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
    case HEATING_MODE_OFF: return "Off";
    case HEATING_MODE_AUTO: return "Auto";
    case HEATING_MODE_COOL: return "Cool";
    case HEATING_MODE_HEAT: return "Heat";
    case HEATING_MODE_EHEAT: return "Emergency Heat";
    default: return "Unknown";
  }
}

const char* WaterFurnaceAurora::get_fan_mode_string(FanMode mode) {
  switch (mode) {
    case FAN_MODE_AUTO: return "Auto";
    case FAN_MODE_CONTINUOUS: return "Continuous";
    case FAN_MODE_INTERMITTENT: return "Intermittent";
    default: return "Unknown";
  }
}

std::string WaterFurnaceAurora::get_current_mode_string() {
  // Based on abc_client.rb refresh() method
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
  
  // Check anti-short-cycle delay (register 6)
  auto it = this->register_cache_.find(registers::COMPRESSOR_ANTI_SHORT_CYCLE);
  if (it != this->register_cache_.end() && it->second != 0) {
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
  
  // Check for AXB by reading register 806 (AXB installed status)
  // Value of 3 means removed/missing
  std::vector<uint16_t> result;
  if (this->read_holding_registers(registers::AXB_INSTALLED, 1, result) && !result.empty()) {
    this->has_axb_ = (result[0] != 3);
    ESP_LOGD(TAG, "AXB detected: %s (register 806 = %d)", this->has_axb_ ? "yes" : "no", result[0]);
  }
  
  // Check for VS Drive by trying to read compressor speed (register 3001)
  if (this->read_holding_registers(registers::COMPRESSOR_SPEED_ACTUAL, 1, result) && !result.empty()) {
    this->has_vs_drive_ = true;
    ESP_LOGD(TAG, "VS Drive detected");
  }
  
  // Check for IZ2 (IntelliZone 2) by reading register 812
  // Value != 0 means IZ2 is installed
  if (this->read_holding_registers(registers::IZ2_INSTALLED, 1, result) && !result.empty()) {
    this->has_iz2_ = (result[0] != 0);
    if (this->has_iz2_) {
      // Read number of zones from register 483
      if (this->read_holding_registers(registers::IZ2_NUM_ZONES, 1, result) && !result.empty()) {
        this->num_iz2_zones_ = result[0];
        if (this->num_iz2_zones_ > MAX_IZ2_ZONES) {
          this->num_iz2_zones_ = MAX_IZ2_ZONES;
        }
      }
      ESP_LOGD(TAG, "IZ2 detected with %d zones", this->num_iz2_zones_);
    }
  }
  
  // Read model number (registers 92-103, 12 registers)
  if (this->read_holding_registers(registers::MODEL_NUMBER, 12, result) && result.size() >= 12) {
    this->model_number_ = registers_to_string(result);
    ESP_LOGD(TAG, "Model number: %s", this->model_number_.c_str());
    if (this->model_number_sensor_ != nullptr) {
      this->model_number_sensor_->publish_state(this->model_number_);
    }
  }
  
  // Read serial number (registers 105-109, 5 registers)
  if (this->read_holding_registers(registers::SERIAL_NUMBER, 5, result) && result.size() >= 5) {
    this->serial_number_ = registers_to_string(result);
    ESP_LOGD(TAG, "Serial number: %s", this->serial_number_.c_str());
    if (this->serial_number_sensor_ != nullptr) {
      this->serial_number_sensor_->publish_state(this->serial_number_);
    }
  }
}

void WaterFurnaceAurora::loop() {
  // Nothing to do in loop - all communication happens in update()
}

void WaterFurnaceAurora::update() {
  ESP_LOGD(TAG, "Updating WaterFurnace Aurora data...");
  this->refresh_all_data();
}

void WaterFurnaceAurora::dump_config() {
  ESP_LOGCONFIG(TAG, "WaterFurnace Aurora:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Flow Control Pin: %s", this->flow_control_pin_ != nullptr ? "configured" : "not configured");
  if (!this->model_number_.empty()) {
    ESP_LOGCONFIG(TAG, "  Model: %s", this->model_number_.c_str());
  }
  if (!this->serial_number_.empty()) {
    ESP_LOGCONFIG(TAG, "  Serial: %s", this->serial_number_.c_str());
  }
  ESP_LOGCONFIG(TAG, "  AXB Present: %s", this->has_axb_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  VS Drive: %s", this->has_vs_drive_ ? "yes" : "no");
  ESP_LOGCONFIG(TAG, "  IZ2 Present: %s", this->has_iz2_ ? "yes" : "no");
  if (this->has_iz2_) {
    ESP_LOGCONFIG(TAG, "  IZ2 Zones: %d", this->num_iz2_zones_);
  }
}

void WaterFurnaceAurora::refresh_all_data() {
  // Build list of registers to read based on Ruby code in abc_client.rb
  // Use the custom 'B' function (0x42) to read specific non-contiguous registers
  
  std::vector<uint16_t> addresses_to_read;
  
  // Core registers always needed (from abc_client.rb @registers_to_read)
  addresses_to_read.push_back(registers::COMPRESSOR_ANTI_SHORT_CYCLE);  // 6
  addresses_to_read.push_back(registers::FP1_TEMP);                      // 19
  addresses_to_read.push_back(registers::FP2_TEMP);                      // 20
  addresses_to_read.push_back(registers::LAST_FAULT);                    // 25
  addresses_to_read.push_back(registers::SYSTEM_OUTPUTS);                // 30
  addresses_to_read.push_back(registers::SYSTEM_STATUS);                 // 31
  addresses_to_read.push_back(registers::LINE_VOLTAGE_SETTING);          // 112
  
  // Thermostat data
  addresses_to_read.push_back(registers::AMBIENT_TEMP);                  // 502
  
  // Temperature registers
  if (this->has_axb_) {
    addresses_to_read.push_back(registers::ENTERING_AIR_AWL);            // 740
    addresses_to_read.push_back(registers::RELATIVE_HUMIDITY);           // 741
    addresses_to_read.push_back(registers::OUTDOOR_TEMP);                // 742
    addresses_to_read.push_back(registers::LEAVING_AIR);                 // 900
    addresses_to_read.push_back(registers::AXB_OUTPUTS);                 // 1104
    addresses_to_read.push_back(registers::LEAVING_WATER);               // 1110
    addresses_to_read.push_back(registers::ENTERING_WATER);              // 1111
  } else {
    addresses_to_read.push_back(registers::ENTERING_AIR);                // 567
  }
  
  // Setpoints
  addresses_to_read.push_back(registers::HEATING_SETPOINT);              // 745
  addresses_to_read.push_back(registers::COOLING_SETPOINT);              // 746
  
  // DHW if AXB present
  if (this->has_axb_) {
    addresses_to_read.push_back(registers::DHW_ENABLED);                 // 400
    addresses_to_read.push_back(registers::DHW_SETPOINT);                // 401
    addresses_to_read.push_back(registers::DHW_TEMP);                    // 1114
    addresses_to_read.push_back(registers::WATERFLOW);                   // 1117
    addresses_to_read.push_back(registers::LOOP_PRESSURE);               // 1119
    // Refrigeration monitoring (requires AXB)
    addresses_to_read.push_back(registers::HEATING_LIQUID_LINE_TEMP);    // 1109
    addresses_to_read.push_back(registers::SATURATED_CONDENSER_TEMP);    // 1134
    addresses_to_read.push_back(registers::SUBCOOL_HEATING);             // 1135
    addresses_to_read.push_back(registers::SUBCOOL_COOLING);             // 1136
    addresses_to_read.push_back(registers::HEAT_OF_EXTRACTION);          // 1154
    addresses_to_read.push_back(registers::HEAT_OF_EXTRACTION + 1);      // 1155
    addresses_to_read.push_back(registers::HEAT_OF_REJECTION);           // 1156
    addresses_to_read.push_back(registers::HEAT_OF_REJECTION + 1);       // 1157
    // VS Pump (requires AXB)
    addresses_to_read.push_back(registers::VS_PUMP_MIN);                 // 321
    addresses_to_read.push_back(registers::VS_PUMP_MAX);                 // 322
    addresses_to_read.push_back(registers::VS_PUMP_SPEED);               // 325
  }
  
  // Blower/ECM registers (always read)
  addresses_to_read.push_back(registers::BLOWER_ONLY_SPEED);             // 340
  addresses_to_read.push_back(registers::LO_COMPRESSOR_ECM_SPEED);       // 341
  addresses_to_read.push_back(registers::HI_COMPRESSOR_ECM_SPEED);       // 342
  addresses_to_read.push_back(registers::ECM_SPEED);                     // 344
  addresses_to_read.push_back(registers::AUX_HEAT_ECM_SPEED);            // 347
  
  // Energy monitoring
  addresses_to_read.push_back(registers::LINE_VOLTAGE);                  // 16
  addresses_to_read.push_back(registers::COMPRESSOR_WATTS);              // 1146
  addresses_to_read.push_back(registers::COMPRESSOR_WATTS + 1);          // 1147
  addresses_to_read.push_back(registers::BLOWER_WATTS);                  // 1148
  addresses_to_read.push_back(registers::BLOWER_WATTS + 1);              // 1149
  addresses_to_read.push_back(registers::AUX_WATTS);                     // 1150
  addresses_to_read.push_back(registers::AUX_WATTS + 1);                 // 1151
  addresses_to_read.push_back(registers::TOTAL_WATTS);                   // 1152
  addresses_to_read.push_back(registers::TOTAL_WATTS + 1);               // 1153
  addresses_to_read.push_back(registers::PUMP_WATTS);                    // 1164
  addresses_to_read.push_back(registers::PUMP_WATTS + 1);                // 1165
  
  // Mode configuration
  addresses_to_read.push_back(registers::FAN_CONFIG);                    // 12005
  addresses_to_read.push_back(registers::HEATING_MODE_READ);             // 12006
  
  // Humidistat (from humidistat.rb)
  // For non-IZ2 systems: registers 12309-12310
  // For IZ2 systems: registers 21114, 31109-31110
  addresses_to_read.push_back(registers::HUMIDISTAT_SETTINGS);           // 12309
  addresses_to_read.push_back(registers::HUMIDISTAT_TARGETS);            // 12310
  
  // VS Drive data
  if (this->has_vs_drive_) {
    addresses_to_read.push_back(362);                                    // Active dehumidify
    addresses_to_read.push_back(registers::COMPRESSOR_SPEED_DESIRED);    // 3000
    addresses_to_read.push_back(registers::COMPRESSOR_SPEED_ACTUAL);     // 3001
    addresses_to_read.push_back(registers::VS_DISCHARGE_PRESSURE);       // 3322
    addresses_to_read.push_back(registers::VS_SUCTION_PRESSURE);         // 3323
    addresses_to_read.push_back(registers::VS_DISCHARGE_TEMP);           // 3325
    addresses_to_read.push_back(registers::VS_DRIVE_TEMP);               // 3327
    addresses_to_read.push_back(registers::VS_INVERTER_TEMP);            // 3522
    addresses_to_read.push_back(registers::VS_EEV_OPEN);                 // 3808
    addresses_to_read.push_back(registers::VS_SUCTION_TEMP);             // 3903
    addresses_to_read.push_back(registers::VS_SUPERHEAT_TEMP);           // 3906
  }
  
  // IZ2 Zone data (from iz2_zone.rb)
  if (this->has_iz2_ && this->num_iz2_zones_ > 0) {
    addresses_to_read.push_back(registers::IZ2_OUTDOOR_TEMP);            // 31003
    addresses_to_read.push_back(registers::IZ2_DEMAND);                  // 31005
    
    for (uint8_t zone = 1; zone <= this->num_iz2_zones_; zone++) {
      // Zone ambient temperature: 31007 + (zone-1)*3
      addresses_to_read.push_back(registers::IZ2_AMBIENT_BASE + ((zone - 1) * 3));
      // Zone config1 (fan, cooling setpoint, partial heating): 31008 + (zone-1)*3
      addresses_to_read.push_back(registers::IZ2_CONFIG1_BASE + ((zone - 1) * 3));
      // Zone config2 (call, mode, damper, heating setpoint): 31009 + (zone-1)*3
      addresses_to_read.push_back(registers::IZ2_CONFIG2_BASE + ((zone - 1) * 3));
      // Zone config3 (priority, size): 31200 + (zone-1)*3
      addresses_to_read.push_back(registers::IZ2_CONFIG3_BASE + ((zone - 1) * 3));
    }
  }
  
  // Read all registers using custom 'B' function
  std::map<uint16_t, uint16_t> result;
  if (!this->read_specific_registers(addresses_to_read, result)) {
    ESP_LOGW(TAG, "Failed to read registers from Aurora");
    return;
  }
  
  // Store in cache
  this->register_cache_ = result;
  
  // Parse and publish system status
  auto it = result.find(registers::LAST_FAULT);
  if (it != result.end()) {
    this->current_fault_ = it->second & 0x7FFF;
    this->locked_out_ = (it->second & 0x8000) != 0;
    
    if (this->fault_code_sensor_ != nullptr) {
      this->fault_code_sensor_->publish_state(this->current_fault_);
    }
    if (this->fault_description_sensor_ != nullptr) {
      this->fault_description_sensor_->publish_state(get_fault_description(this->current_fault_));
    }
    if (this->lockout_sensor_ != nullptr) {
      this->lockout_sensor_->publish_state(this->locked_out_);
    }
  }
  
  // System outputs (register 30)
  it = result.find(registers::SYSTEM_OUTPUTS);
  if (it != result.end()) {
    this->system_outputs_ = it->second;
    
    if (this->compressor_sensor_ != nullptr) {
      this->compressor_sensor_->publish_state((this->system_outputs_ & (OUTPUT_CC | OUTPUT_CC2)) != 0);
    }
    if (this->blower_sensor_ != nullptr) {
      this->blower_sensor_->publish_state((this->system_outputs_ & OUTPUT_BLOWER) != 0);
    }
    if (this->aux_heat_sensor_ != nullptr) {
      this->aux_heat_sensor_->publish_state((this->system_outputs_ & (OUTPUT_EH1 | OUTPUT_EH2)) != 0);
    }
  }
  
  // System status (register 31) - LPS, HPS, emergency shutdown, load shed
  it = result.find(registers::SYSTEM_STATUS);
  if (it != result.end()) {
    uint16_t status = it->second;
    
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
  
  // AXB outputs (register 1104)
  it = result.find(registers::AXB_OUTPUTS);
  if (it != result.end()) {
    this->axb_outputs_ = it->second;
    
    if (this->dhw_running_sensor_ != nullptr) {
      this->dhw_running_sensor_->publish_state((this->axb_outputs_ & AXB_OUTPUT_DHW) != 0);
    }
    if (this->loop_pump_sensor_ != nullptr) {
      this->loop_pump_sensor_->publish_state((this->axb_outputs_ & AXB_OUTPUT_LOOP_PUMP) != 0);
    }
  }
  
  // Active dehumidify (register 362)
  it = result.find(362);
  if (it != result.end()) {
    this->active_dehumidify_ = (it->second != 0);
  }
  
  // Current mode
  if (this->current_mode_sensor_ != nullptr) {
    this->current_mode_sensor_->publish_state(this->get_current_mode_string());
  }
  
  // Temperatures - all stored as signed value / 10
  it = result.find(registers::AMBIENT_TEMP);
  if (it != result.end()) {
    this->ambient_temp_ = to_signed_tenths(it->second);
    if (this->ambient_temp_sensor_ != nullptr) {
      this->ambient_temp_sensor_->publish_state(this->ambient_temp_);
    }
  }
  
  // Entering air - use AWL register if AXB present
  uint16_t entering_air_reg = this->has_axb_ ? registers::ENTERING_AIR_AWL : registers::ENTERING_AIR;
  it = result.find(entering_air_reg);
  if (it != result.end() && this->entering_air_sensor_ != nullptr) {
    this->entering_air_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  it = result.find(registers::LEAVING_AIR);
  if (it != result.end() && this->leaving_air_sensor_ != nullptr) {
    this->leaving_air_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  it = result.find(registers::OUTDOOR_TEMP);
  if (it != result.end() && this->outdoor_temp_sensor_ != nullptr) {
    this->outdoor_temp_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  it = result.find(registers::LEAVING_WATER);
  if (it != result.end() && this->leaving_water_sensor_ != nullptr) {
    this->leaving_water_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  it = result.find(registers::ENTERING_WATER);
  if (it != result.end() && this->entering_water_sensor_ != nullptr) {
    this->entering_water_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  // Humidity
  it = result.find(registers::RELATIVE_HUMIDITY);
  if (it != result.end() && this->humidity_sensor_ != nullptr) {
    this->humidity_sensor_->publish_state(it->second);
  }
  
  // Setpoints - stored as value / 10
  it = result.find(registers::HEATING_SETPOINT);
  if (it != result.end()) {
    this->heating_setpoint_ = to_tenths(it->second);
    if (this->heating_setpoint_sensor_ != nullptr) {
      this->heating_setpoint_sensor_->publish_state(this->heating_setpoint_);
    }
  }
  
  it = result.find(registers::COOLING_SETPOINT);
  if (it != result.end()) {
    this->cooling_setpoint_ = to_tenths(it->second);
    if (this->cooling_setpoint_sensor_ != nullptr) {
      this->cooling_setpoint_sensor_->publish_state(this->cooling_setpoint_);
    }
  }
  
  // DHW
  it = result.find(registers::DHW_ENABLED);
  if (it != result.end()) {
    this->dhw_enabled_ = (it->second != 0);
  }
  
  it = result.find(registers::DHW_SETPOINT);
  if (it != result.end()) {
    this->dhw_setpoint_ = to_tenths(it->second);
    if (this->dhw_setpoint_sensor_ != nullptr) {
      this->dhw_setpoint_sensor_->publish_state(this->dhw_setpoint_);
    }
  }
  
  it = result.find(registers::DHW_TEMP);
  if (it != result.end()) {
    this->dhw_temp_ = to_signed_tenths(it->second);
    if (this->dhw_temp_sensor_ != nullptr) {
      this->dhw_temp_sensor_->publish_state(this->dhw_temp_);
    }
  }
  
  // HVAC mode (register 12006) - bits 8-10 contain mode
  it = result.find(registers::HEATING_MODE_READ);
  if (it != result.end()) {
    uint8_t mode_val = (it->second >> 8) & 0x07;
    this->hvac_mode_ = static_cast<HeatingMode>(mode_val);
    if (this->hvac_mode_sensor_ != nullptr) {
      this->hvac_mode_sensor_->publish_state(get_hvac_mode_string(this->hvac_mode_));
    }
  }
  
  // Fan mode (register 12005)
  it = result.find(registers::FAN_CONFIG);
  if (it != result.end()) {
    uint16_t config = it->second;
    if (config & 0x80) {
      this->fan_mode_ = FAN_MODE_CONTINUOUS;
    } else if (config & 0x100) {
      this->fan_mode_ = FAN_MODE_INTERMITTENT;
    } else {
      this->fan_mode_ = FAN_MODE_AUTO;
    }
    if (this->fan_mode_sensor_ != nullptr) {
      this->fan_mode_sensor_->publish_state(get_fan_mode_string(this->fan_mode_));
    }
  }
  
  // Line voltage
  it = result.find(registers::LINE_VOLTAGE);
  if (it != result.end() && this->line_voltage_sensor_ != nullptr) {
    this->line_voltage_sensor_->publish_state(it->second);
  }
  
  // Power - 32-bit values (high word, low word)
  auto it_high = result.find(registers::TOTAL_WATTS);
  auto it_low = result.find(registers::TOTAL_WATTS + 1);
  if (it_high != result.end() && it_low != result.end() && this->total_watts_sensor_ != nullptr) {
    this->total_watts_sensor_->publish_state(to_uint32(it_high->second, it_low->second));
  }
  
  it_high = result.find(registers::COMPRESSOR_WATTS);
  it_low = result.find(registers::COMPRESSOR_WATTS + 1);
  if (it_high != result.end() && it_low != result.end() && this->compressor_watts_sensor_ != nullptr) {
    this->compressor_watts_sensor_->publish_state(to_uint32(it_high->second, it_low->second));
  }
  
  it_high = result.find(registers::BLOWER_WATTS);
  it_low = result.find(registers::BLOWER_WATTS + 1);
  if (it_high != result.end() && it_low != result.end() && this->blower_watts_sensor_ != nullptr) {
    this->blower_watts_sensor_->publish_state(to_uint32(it_high->second, it_low->second));
  }
  
  it_high = result.find(registers::AUX_WATTS);
  it_low = result.find(registers::AUX_WATTS + 1);
  if (it_high != result.end() && it_low != result.end() && this->aux_watts_sensor_ != nullptr) {
    this->aux_watts_sensor_->publish_state(to_uint32(it_high->second, it_low->second));
  }
  
  it_high = result.find(registers::PUMP_WATTS);
  it_low = result.find(registers::PUMP_WATTS + 1);
  if (it_high != result.end() && it_low != result.end() && this->pump_watts_sensor_ != nullptr) {
    this->pump_watts_sensor_->publish_state(to_uint32(it_high->second, it_low->second));
  }
  
  // Loop
  it = result.find(registers::WATERFLOW);
  if (it != result.end() && this->waterflow_sensor_ != nullptr) {
    this->waterflow_sensor_->publish_state(to_tenths(it->second));
  }
  
  it = result.find(registers::LOOP_PRESSURE);
  if (it != result.end() && it->second < 10000 && this->loop_pressure_sensor_ != nullptr) {
    this->loop_pressure_sensor_->publish_state(to_tenths(it->second));
  }
  
  // VS Drive
  it = result.find(registers::COMPRESSOR_SPEED_ACTUAL);
  if (it != result.end() && this->compressor_speed_sensor_ != nullptr) {
    this->compressor_speed_sensor_->publish_state(it->second);
  }
  
  it = result.find(registers::VS_DISCHARGE_PRESSURE);
  if (it != result.end() && this->discharge_pressure_sensor_ != nullptr) {
    this->discharge_pressure_sensor_->publish_state(to_tenths(it->second));
  }
  
  it = result.find(registers::VS_SUCTION_PRESSURE);
  if (it != result.end() && this->suction_pressure_sensor_ != nullptr) {
    this->suction_pressure_sensor_->publish_state(to_tenths(it->second));
  }
  
  it = result.find(registers::VS_EEV_OPEN);
  if (it != result.end() && this->eev_open_sensor_ != nullptr) {
    this->eev_open_sensor_->publish_state(it->second);
  }
  
  it = result.find(registers::VS_SUPERHEAT_TEMP);
  if (it != result.end() && this->superheat_sensor_ != nullptr) {
    this->superheat_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  // Additional VS Drive temperatures
  it = result.find(registers::COMPRESSOR_SPEED_DESIRED);
  if (it != result.end() && this->compressor_desired_speed_sensor_ != nullptr) {
    this->compressor_desired_speed_sensor_->publish_state(it->second);
  }
  
  it = result.find(registers::VS_DISCHARGE_TEMP);
  if (it != result.end() && this->discharge_temp_sensor_ != nullptr) {
    this->discharge_temp_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  it = result.find(registers::VS_SUCTION_TEMP);
  if (it != result.end() && this->suction_temp_sensor_ != nullptr) {
    this->suction_temp_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  it = result.find(registers::VS_DRIVE_TEMP);
  if (it != result.end() && this->vs_drive_temp_sensor_ != nullptr) {
    this->vs_drive_temp_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  it = result.find(registers::VS_INVERTER_TEMP);
  if (it != result.end() && this->vs_inverter_temp_sensor_ != nullptr) {
    this->vs_inverter_temp_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  // FP1 - Cooling liquid line temperature (register 19)
  it = result.find(registers::FP1_TEMP);
  if (it != result.end() && this->fp1_sensor_ != nullptr) {
    this->fp1_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  // FP2 - Air coil temperature (register 20)
  it = result.find(registers::FP2_TEMP);
  if (it != result.end() && this->fp2_sensor_ != nullptr) {
    this->fp2_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  // Line voltage setting (register 112)
  it = result.find(registers::LINE_VOLTAGE_SETTING);
  if (it != result.end() && this->line_voltage_setting_sensor_ != nullptr) {
    this->line_voltage_setting_sensor_->publish_state(it->second);
  }
  
  // Anti-short-cycle countdown (register 6)
  it = result.find(registers::COMPRESSOR_ANTI_SHORT_CYCLE);
  if (it != result.end() && this->anti_short_cycle_sensor_ != nullptr) {
    this->anti_short_cycle_sensor_->publish_state(it->second);
  }
  
  // Blower/ECM sensors
  it = result.find(registers::ECM_SPEED);
  if (it != result.end() && this->blower_speed_sensor_ != nullptr) {
    this->blower_speed_sensor_->publish_state(it->second);
  }
  
  it = result.find(registers::BLOWER_ONLY_SPEED);
  if (it != result.end() && this->blower_only_speed_sensor_ != nullptr) {
    this->blower_only_speed_sensor_->publish_state(it->second);
  }
  
  it = result.find(registers::LO_COMPRESSOR_ECM_SPEED);
  if (it != result.end() && this->lo_compressor_speed_sensor_ != nullptr) {
    this->lo_compressor_speed_sensor_->publish_state(it->second);
  }
  
  it = result.find(registers::HI_COMPRESSOR_ECM_SPEED);
  if (it != result.end() && this->hi_compressor_speed_sensor_ != nullptr) {
    this->hi_compressor_speed_sensor_->publish_state(it->second);
  }
  
  it = result.find(registers::AUX_HEAT_ECM_SPEED);
  if (it != result.end() && this->aux_heat_speed_sensor_ != nullptr) {
    this->aux_heat_speed_sensor_->publish_state(it->second);
  }
  
  // VS Pump sensors
  it = result.find(registers::VS_PUMP_SPEED);
  if (it != result.end() && this->pump_speed_sensor_ != nullptr) {
    this->pump_speed_sensor_->publish_state(it->second);
  }
  
  it = result.find(registers::VS_PUMP_MIN);
  if (it != result.end() && this->pump_min_speed_sensor_ != nullptr) {
    this->pump_min_speed_sensor_->publish_state(it->second);
  }
  
  it = result.find(registers::VS_PUMP_MAX);
  if (it != result.end() && this->pump_max_speed_sensor_ != nullptr) {
    this->pump_max_speed_sensor_->publish_state(it->second);
  }
  
  // Refrigeration monitoring sensors
  it = result.find(registers::HEATING_LIQUID_LINE_TEMP);
  if (it != result.end() && this->heating_liquid_line_temp_sensor_ != nullptr) {
    this->heating_liquid_line_temp_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  it = result.find(registers::SATURATED_CONDENSER_TEMP);
  if (it != result.end() && this->saturated_condenser_temp_sensor_ != nullptr) {
    this->saturated_condenser_temp_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  // Subcool temperature - use cooling or heating register based on reversing valve state
  // From Ruby compressor.rb: @subcool_temperature = registers[registers[30].include?(:rv) ? 1136 : 1135]
  bool is_cooling = (this->system_outputs_ & OUTPUT_RV) != 0;
  uint16_t subcool_reg = is_cooling ? registers::SUBCOOL_COOLING : registers::SUBCOOL_HEATING;
  it = result.find(subcool_reg);
  if (it != result.end() && this->subcool_temp_sensor_ != nullptr) {
    this->subcool_temp_sensor_->publish_state(to_signed_tenths(it->second));
  }
  
  // Heat of extraction (32-bit, signed)
  it_high = result.find(registers::HEAT_OF_EXTRACTION);
  it_low = result.find(registers::HEAT_OF_EXTRACTION + 1);
  if (it_high != result.end() && it_low != result.end() && this->heat_of_extraction_sensor_ != nullptr) {
    this->heat_of_extraction_sensor_->publish_state(to_int32(it_high->second, it_low->second));
  }
  
  // Heat of rejection (32-bit, signed)
  it_high = result.find(registers::HEAT_OF_REJECTION);
  it_low = result.find(registers::HEAT_OF_REJECTION + 1);
  if (it_high != result.end() && it_low != result.end() && this->heat_of_rejection_sensor_ != nullptr) {
    this->heat_of_rejection_sensor_->publish_state(to_int32(it_high->second, it_low->second));
  }
  
  // Humidifier/Dehumidifier status
  // From humidistat.rb: humidifier_running = humidifier? && outputs.include?(:accessory)
  // Accessory bit in register 30 is 0x200
  if (this->humidifier_running_sensor_ != nullptr) {
    bool humidifier_running = (this->system_outputs_ & OUTPUT_ACCESSORY) != 0;
    this->humidifier_running_sensor_->publish_state(humidifier_running);
  }
  
  // Dehumidifier can run via AXB accessory2 bit (0x10 in register 1104)
  // or via VS drive active dehumidify mode (register 362)
  if (this->dehumidifier_running_sensor_ != nullptr) {
    bool dehumidifier_running = this->active_dehumidify_ || 
                                 ((this->axb_outputs_ & 0x10) != 0);  // AXB_OUTPUT_ACCESSORY2
    this->dehumidifier_running_sensor_->publish_state(dehumidifier_running);
  }
  
  // Humidification/Dehumidification targets from register 12310
  // Format: high byte = humidification target, low byte = dehumidification target
  it = result.find(registers::HUMIDISTAT_TARGETS);
  if (it != result.end()) {
    if (this->humidification_target_sensor_ != nullptr) {
      uint8_t humidification_target = (it->second >> 8) & 0xFF;
      this->humidification_target_sensor_->publish_state(humidification_target);
    }
    if (this->dehumidification_target_sensor_ != nullptr) {
      uint8_t dehumidification_target = it->second & 0xFF;
      this->dehumidification_target_sensor_->publish_state(dehumidification_target);
    }
  }
  
  // Parse IZ2 zone data (from iz2_zone.rb)
  if (this->has_iz2_ && this->num_iz2_zones_ > 0) {
    for (uint8_t zone = 1; zone <= this->num_iz2_zones_; zone++) {
      IZ2ZoneData& zone_data = this->iz2_zones_[zone - 1];
      
      // Ambient temperature: 31007 + (zone-1)*3
      it = result.find(registers::IZ2_AMBIENT_BASE + ((zone - 1) * 3));
      if (it != result.end()) {
        zone_data.ambient_temperature = to_signed_tenths(it->second);
      }
      
      // Config1: fan mode, on/off times, cooling setpoint (partial)
      // Register 31008 + (zone-1)*3
      auto it_config1 = result.find(registers::IZ2_CONFIG1_BASE + ((zone - 1) * 3));
      if (it_config1 != result.end()) {
        uint16_t config1 = it_config1->second;
        
        // Fan mode: bit 7 = continuous, bit 8 = intermittent
        if (config1 & 0x80) {
          zone_data.target_fan_mode = FAN_MODE_CONTINUOUS;
        } else if (config1 & 0x100) {
          zone_data.target_fan_mode = FAN_MODE_INTERMITTENT;
        } else {
          zone_data.target_fan_mode = FAN_MODE_AUTO;
        }
        
        // Fan on time: bits 9-11, multiply by 5
        zone_data.fan_on_time = ((config1 >> 9) & 0x7) * 5;
        
        // Fan off time: bits 12-14, add 1 then multiply by 5
        zone_data.fan_off_time = (((config1 >> 12) & 0x7) + 1) * 5;
        
        // Cooling setpoint: bits 1-6, add 36
        zone_data.cooling_setpoint = ((config1 & 0x7e) >> 1) + 36.0f;
      }
      
      // Config2: call, mode, damper, heating setpoint
      // Register 31009 + (zone-1)*3
      auto it_config2 = result.find(registers::IZ2_CONFIG2_BASE + ((zone - 1) * 3));
      if (it_config2 != result.end() && it_config1 != result.end()) {
        uint16_t config2 = it_config2->second;
        uint16_t config1 = it_config1->second;
        
        // Current call: bits 1-3
        uint8_t call_val = (config2 >> 1) & 0x7;
        zone_data.current_call = static_cast<ZoneCall>(call_val);
        
        // Target mode: bits 8-9
        uint8_t mode_val = (config2 >> 8) & 0x3;
        zone_data.target_mode = static_cast<HeatingMode>(mode_val);
        
        // Damper: bit 4
        zone_data.damper_open = (config2 & 0x10) != 0;
        
        // Heating setpoint: bits 11-15 from config2, bit 0 from config1 as carry
        uint8_t carry = config1 & 0x01;
        zone_data.heating_setpoint = ((carry << 5) | ((config2 & 0xf800) >> 11)) + 36.0f;
      }
      
      // Config3: priority and size
      // Register 31200 + (zone-1)*3
      auto it_config3 = result.find(registers::IZ2_CONFIG3_BASE + ((zone - 1) * 3));
      if (it_config3 != result.end()) {
        uint16_t config3 = it_config3->second;
        
        // Priority: bit 5
        zone_data.priority = (config3 & 0x20) ? ZONE_PRIORITY_ECONOMY : ZONE_PRIORITY_COMFORT;
        
        // Size: bits 3-4
        uint8_t size_val = (config3 >> 3) & 0x3;
        zone_data.size = static_cast<ZoneSize>(size_val);
        
        // Normalized size: high byte
        zone_data.normalized_size = (config3 >> 8) & 0xFF;
      }
      
      ESP_LOGV(TAG, "Zone %d: temp=%.1f, heat_sp=%.1f, cool_sp=%.1f, mode=%d, call=%d, damper=%s",
               zone, zone_data.ambient_temperature, zone_data.heating_setpoint, 
               zone_data.cooling_setpoint, zone_data.target_mode, zone_data.current_call,
               zone_data.damper_open ? "open" : "closed");
    }
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

// Blower speed controls (from blower.rb)
// ECM blower speeds are 1-12
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

// Pump speed controls (from pump.rb)
// VS pump speeds are 1-100 (percent)
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

// Fan intermittent timing (from thermostat.rb)
// On time: 0, 5, 10, 15, 20, 25 (minutes, must be multiple of 5)
// Off time: 5, 10, 15, 20, 25, 30, 35, 40 (minutes, must be multiple of 5)
bool WaterFurnaceAurora::set_fan_intermittent_on(uint8_t minutes) {
  if (minutes > 25 || (minutes % 5) != 0) {
    ESP_LOGW(TAG, "Fan intermittent on time %d invalid (0, 5, 10, 15, 20, 25)", minutes);
    return false;
  }
  return this->write_holding_register(12622, minutes);  // Register 12622
}

bool WaterFurnaceAurora::set_fan_intermittent_off(uint8_t minutes) {
  if (minutes < 5 || minutes > 40 || (minutes % 5) != 0) {
    ESP_LOGW(TAG, "Fan intermittent off time %d invalid (5, 10, 15, 20, 25, 30, 35, 40)", minutes);
    return false;
  }
  return this->write_holding_register(12623, minutes);  // Register 12623
}

// Humidifier controls (from humidistat.rb)
// Humidification target: 15-50%
// Dehumidification target: 35-65%
bool WaterFurnaceAurora::set_humidification_target(uint8_t percent) {
  if (percent < 15 || percent > 50) {
    ESP_LOGW(TAG, "Humidification target %d out of range (15-50)", percent);
    return false;
  }
  // Read current targets register first to preserve dehumidification target
  std::vector<uint16_t> result;
  if (!this->read_holding_registers(registers::HUMIDISTAT_TARGETS, 1, result) || result.empty()) {
    ESP_LOGW(TAG, "Failed to read current humidistat targets");
    return false;
  }
  uint8_t dehum_target = result[0] & 0xFF;
  uint16_t value = (percent << 8) | dehum_target;
  return this->write_holding_register(registers::HUMIDISTAT_TARGETS, value);
}

bool WaterFurnaceAurora::set_dehumidification_target(uint8_t percent) {
  if (percent < 35 || percent > 65) {
    ESP_LOGW(TAG, "Dehumidification target %d out of range (35-65)", percent);
    return false;
  }
  // Read current targets register first to preserve humidification target
  std::vector<uint16_t> result;
  if (!this->read_holding_registers(registers::HUMIDISTAT_TARGETS, 1, result) || result.empty()) {
    ESP_LOGW(TAG, "Failed to read current humidistat targets");
    return false;
  }
  uint8_t hum_target = (result[0] >> 8) & 0xFF;
  uint16_t value = (hum_target << 8) | percent;
  return this->write_holding_register(registers::HUMIDISTAT_TARGETS, value);
}

// Clear fault history (from abc_client.rb)
bool WaterFurnaceAurora::clear_fault_history() {
  ESP_LOGI(TAG, "Clearing fault history");
  return this->write_holding_register(47, 0x5555);  // Magic value to clear faults
}

// IZ2 Zone control methods (from iz2_zone.rb)
// Zone numbers are 1-6

bool WaterFurnaceAurora::set_zone_heating_setpoint(uint8_t zone_number, float temp) {
  if (zone_number < 1 || zone_number > MAX_IZ2_ZONES) {
    ESP_LOGW(TAG, "Invalid zone number %d (1-6)", zone_number);
    return false;
  }
  if (temp < 40.0f || temp > 90.0f) {
    ESP_LOGW(TAG, "Heating setpoint %.1f out of range (40-90)", temp);
    return false;
  }
  // Register 21203 + (zone - 1) * 9
  uint16_t reg = registers::IZ2_HEAT_SP_WRITE_BASE + ((zone_number - 1) * 9);
  uint16_t value = static_cast<uint16_t>(temp * 10);
  return this->write_holding_register(reg, value);
}

bool WaterFurnaceAurora::set_zone_cooling_setpoint(uint8_t zone_number, float temp) {
  if (zone_number < 1 || zone_number > MAX_IZ2_ZONES) {
    ESP_LOGW(TAG, "Invalid zone number %d (1-6)", zone_number);
    return false;
  }
  if (temp < 54.0f || temp > 99.0f) {
    ESP_LOGW(TAG, "Cooling setpoint %.1f out of range (54-99)", temp);
    return false;
  }
  // Register 21204 + (zone - 1) * 9
  uint16_t reg = registers::IZ2_COOL_SP_WRITE_BASE + ((zone_number - 1) * 9);
  uint16_t value = static_cast<uint16_t>(temp * 10);
  return this->write_holding_register(reg, value);
}

bool WaterFurnaceAurora::set_zone_hvac_mode(uint8_t zone_number, HeatingMode mode) {
  if (zone_number < 1 || zone_number > MAX_IZ2_ZONES) {
    ESP_LOGW(TAG, "Invalid zone number %d (1-6)", zone_number);
    return false;
  }
  // Register 21202 + (zone - 1) * 9
  uint16_t reg = registers::IZ2_MODE_WRITE_BASE + ((zone_number - 1) * 9);
  return this->write_holding_register(reg, static_cast<uint16_t>(mode));
}

bool WaterFurnaceAurora::set_zone_fan_mode(uint8_t zone_number, FanMode mode) {
  if (zone_number < 1 || zone_number > MAX_IZ2_ZONES) {
    ESP_LOGW(TAG, "Invalid zone number %d (1-6)", zone_number);
    return false;
  }
  // Register 21205 + (zone - 1) * 9
  uint16_t reg = registers::IZ2_FAN_MODE_WRITE_BASE + ((zone_number - 1) * 9);
  return this->write_holding_register(reg, static_cast<uint16_t>(mode));
}

bool WaterFurnaceAurora::set_zone_fan_intermittent_on(uint8_t zone_number, uint8_t minutes) {
  if (zone_number < 1 || zone_number > MAX_IZ2_ZONES) {
    ESP_LOGW(TAG, "Invalid zone number %d (1-6)", zone_number);
    return false;
  }
  if (minutes > 25 || (minutes % 5) != 0) {
    ESP_LOGW(TAG, "Fan intermittent on time %d invalid (0, 5, 10, 15, 20, 25)", minutes);
    return false;
  }
  // Register 21206 + (zone - 1) * 9
  uint16_t reg = registers::IZ2_FAN_ON_WRITE_BASE + ((zone_number - 1) * 9);
  return this->write_holding_register(reg, minutes);
}

bool WaterFurnaceAurora::set_zone_fan_intermittent_off(uint8_t zone_number, uint8_t minutes) {
  if (zone_number < 1 || zone_number > MAX_IZ2_ZONES) {
    ESP_LOGW(TAG, "Invalid zone number %d (1-6)", zone_number);
    return false;
  }
  if (minutes < 5 || minutes > 40 || (minutes % 5) != 0) {
    ESP_LOGW(TAG, "Fan intermittent off time %d invalid (5, 10, 15, 20, 25, 30, 35, 40)", minutes);
    return false;
  }
  // Register 21207 + (zone - 1) * 9
  uint16_t reg = registers::IZ2_FAN_OFF_WRITE_BASE + ((zone_number - 1) * 9);
  return this->write_holding_register(reg, minutes);
}

// WaterFurnace custom protocol: Function 0x42 ('B') - Read specific registers
// Based on lib/aurora/modbus/slave.rb read_multiple_holding_registers
// Request format: [addr][0x42][addr1_hi][addr1_lo][addr2_hi][addr2_lo]...[CRC]
// Response format: [addr][0x42][byte_count][data...][CRC]
bool WaterFurnaceAurora::read_specific_registers(const std::vector<uint16_t> &addresses,
                                                   std::map<uint16_t, uint16_t> &result) {
  result.clear();
  
  if (addresses.empty()) {
    return true;
  }
  
  // Retry loop (like Ruby's read_retries: 2)
  for (uint8_t attempt = 0; attempt <= this->read_retries_; attempt++) {
    if (attempt > 0) {
      ESP_LOGD(TAG, "Retry %d for 0x42 read", attempt);
      delay(50);  // Small delay before retry
    }
    
    // Build request: address + function + list of register addresses
    std::vector<uint8_t> request;
    request.push_back(this->address_);
    request.push_back(FUNC_READ_SPECIFIC);  // 0x42 = 'B'
    
    for (uint16_t addr : addresses) {
      request.push_back(addr >> 8);    // High byte
      request.push_back(addr & 0xFF);  // Low byte
    }
    
    // Add CRC
    uint16_t crc = this->calculate_crc(request.data(), request.size());
    request.push_back(crc & 0xFF);   // CRC low byte first (Modbus convention)
    request.push_back(crc >> 8);     // CRC high byte
    
    // Clear any pending data
    while (this->available()) {
      this->read();
    }
    
    // Enable TX mode for RS485
    if (this->flow_control_pin_ != nullptr) {
      this->flow_control_pin_->digital_write(true);
    }
    
    // Send request
    this->write_array(request.data(), request.size());
    this->flush();
    
    // Small delay to ensure last byte is transmitted before switching to RX
    delayMicroseconds(500);
    
    // Disable TX mode (enable RX) for RS485
    if (this->flow_control_pin_ != nullptr) {
      this->flow_control_pin_->digital_write(false);
    }
    
    ESP_LOGV(TAG, "Sent 0x42 request for %d registers", addresses.size());
    
    // Wait for response
    std::vector<uint8_t> response;
    if (!this->wait_for_response(response, 2000)) {
      continue;  // Retry on timeout
    }
    
    // Parse response
    // Response: [addr][func][byte_count][data...][crc_lo][crc_hi]
    if (response.size() < 5) {
      ESP_LOGW(TAG, "Response too short: %d bytes", response.size());
      continue;  // Retry
    }
    
    if (response[0] != this->address_) {
      ESP_LOGW(TAG, "Wrong address in response: 0x%02X", response[0]);
      continue;  // Retry
    }
    
    if (response[1] != FUNC_READ_SPECIFIC) {
      ESP_LOGW(TAG, "Wrong function in response: 0x%02X", response[1]);
      continue;  // Retry
    }
    
    uint8_t byte_count = response[2];
    if (response.size() < static_cast<size_t>(3 + byte_count + 2)) {
      ESP_LOGW(TAG, "Response truncated: expected %d, got %d", 3 + byte_count + 2, response.size());
      continue;  // Retry
    }
    
    // Parse register values (each is 2 bytes, big endian)
    size_t data_start = 3;
    for (size_t i = 0; i < addresses.size() && (i * 2) < byte_count; i++) {
      uint16_t value = (response[data_start + i * 2] << 8) | response[data_start + i * 2 + 1];
      result[addresses[i]] = value;
    }
    
    ESP_LOGV(TAG, "Received %d register values", result.size());
    return true;
  }
  
  ESP_LOGW(TAG, "Failed to read registers after %d retries", this->read_retries_);
  return false;
}

// Function 0x41 ('A') - Read multiple register ranges
// Request: [addr][0x41][start1_hi][start1_lo][count1_hi][count1_lo]...[CRC]
// Response: [addr][0x41][byte_count][data...][CRC]
bool WaterFurnaceAurora::read_register_ranges(const std::vector<std::pair<uint16_t, uint16_t>> &ranges,
                                                std::map<uint16_t, uint16_t> &result) {
  result.clear();
  
  if (ranges.empty()) {
    return true;
  }
  
  // Build request
  std::vector<uint8_t> request;
  request.push_back(this->address_);
  request.push_back(FUNC_READ_RANGES);  // 0x41 = 'A'
  
  for (const auto &range : ranges) {
    request.push_back(range.first >> 8);     // Start address high
    request.push_back(range.first & 0xFF);   // Start address low
    request.push_back(range.second >> 8);    // Count high
    request.push_back(range.second & 0xFF);  // Count low
  }
  
  // Add CRC
  uint16_t crc = this->calculate_crc(request.data(), request.size());
  request.push_back(crc & 0xFF);
  request.push_back(crc >> 8);
  
  // Clear any pending data
  while (this->available()) {
    this->read();
  }
  
  // Enable TX mode for RS485
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(true);
  }
  
  // Send request
  this->write_array(request.data(), request.size());
  this->flush();
  
  // Small delay to ensure last byte is transmitted before switching to RX
  delayMicroseconds(500);
  
  // Disable TX mode (enable RX) for RS485
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(false);
  }
  
  // Wait for response
  std::vector<uint8_t> response;
  if (!this->wait_for_response(response, 2000)) {
    return false;
  }
  
  // Parse response
  if (response.size() < 5 || response[0] != this->address_ || response[1] != FUNC_READ_RANGES) {
    return false;
  }
  
  uint8_t byte_count = response[2];
  size_t data_start = 3;
  size_t data_idx = 0;
  
  // Map response data back to register addresses
  for (const auto &range : ranges) {
    for (uint16_t i = 0; i < range.second && (data_idx * 2) < byte_count; i++) {
      uint16_t value = (response[data_start + data_idx * 2] << 8) | 
                       response[data_start + data_idx * 2 + 1];
      result[range.first + i] = value;
      data_idx++;
    }
  }
  
  return true;
}

// Standard Modbus function 0x03 - Read holding registers
bool WaterFurnaceAurora::read_holding_registers(uint16_t start_addr, uint16_t count, 
                                                  std::vector<uint16_t> &result) {
  result.clear();
  
  // Retry loop (like Ruby's read_retries: 2)
  for (uint8_t attempt = 0; attempt <= this->read_retries_; attempt++) {
    if (attempt > 0) {
      ESP_LOGD(TAG, "Retry %d for 0x03 read at %d", attempt, start_addr);
      delay(50);  // Small delay before retry
    }
    
    // Build request: [addr][0x03][start_hi][start_lo][count_hi][count_lo][CRC]
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
    
    // Clear any pending data
    while (this->available()) {
      this->read();
    }
    
    // Enable TX mode for RS485
    if (this->flow_control_pin_ != nullptr) {
      this->flow_control_pin_->digital_write(true);
    }
    
    // Send request
    this->write_array(request, 8);
    this->flush();
    
    // Small delay to ensure last byte is transmitted before switching to RX
    delayMicroseconds(500);
    
    // Disable TX mode (enable RX) for RS485
    if (this->flow_control_pin_ != nullptr) {
      this->flow_control_pin_->digital_write(false);
    }
    
    // Wait for response
    std::vector<uint8_t> response;
    if (!this->wait_for_response(response, 1000)) {
      continue;  // Retry on timeout
    }
    
    // Parse response
    if (response.size() < 5) {
      continue;  // Retry
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
  // Build request: [addr][0x06][reg_hi][reg_lo][val_hi][val_lo][CRC]
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
  
  // Clear any pending data
  while (this->available()) {
    this->read();
  }
  
  // Enable TX mode for RS485
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(true);
  }
  
  // Send request
  this->write_array(request, 8);
  this->flush();
  
  // Small delay to ensure last byte is transmitted before switching to RX
  delayMicroseconds(500);
  
  // Disable TX mode (enable RX) for RS485
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(false);
  }
  
  ESP_LOGD(TAG, "Writing register %d = %d", addr, value);
  
  // Wait for response (echo of request)
  std::vector<uint8_t> response;
  return this->wait_for_response(response, 1000);
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
  uint32_t start = millis();
  
  while (millis() - start < timeout_ms) {
    while (this->available()) {
      response.push_back(this->read());
    }
    
    // Check if we have a complete response
    if (response.size() >= 5) {
      // Minimum response: address(1) + function(1) + byte_count(1) + crc(2)
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
          size_t expected_len = 3 + byte_count + 2;  // header + data + CRC
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
  
  ESP_LOGW(TAG, "Modbus timeout waiting for response (got %d bytes)", response.size());
  return false;
}

// Data conversion helpers (from registers.rb)
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

// Convert array of registers to string (from registers.rb to_string)
// Each register contains 2 ASCII characters (high byte, low byte)
std::string WaterFurnaceAurora::registers_to_string(const std::vector<uint16_t> &regs) {
  std::string result;
  result.reserve(regs.size() * 2);
  
  for (uint16_t reg : regs) {
    char high_char = static_cast<char>(reg >> 8);
    char low_char = static_cast<char>(reg & 0xFF);
    
    // Stop at null character or space padding
    if (high_char == '\0' || high_char == ' ') break;
    result += high_char;
    
    if (low_char == '\0' || low_char == ' ') break;
    result += low_char;
  }
  
  // Trim trailing spaces
  while (!result.empty() && result.back() == ' ') {
    result.pop_back();
  }
  
  return result;
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
