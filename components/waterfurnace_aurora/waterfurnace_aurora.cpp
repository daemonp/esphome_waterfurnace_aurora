#include "waterfurnace_aurora.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#include <cstring>
#include <algorithm>

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "waterfurnace_aurora";

// ============================================================================
// Helper: current mode string
// ============================================================================

/// Derive the human-readable operating mode from system outputs and cache state.
/// Uses register_cache_ to check anti-short-cycle countdown (determines "Waiting"
/// vs "Standby"). Returns a string literal — no heap allocation.
const char *WaterFurnaceAurora::get_current_mode_string() {
  if (this->system_outputs_ & OUTPUT_LOCKOUT) return "Lockout";
  if (this->active_dehumidify_) return "Dehumidify";
  
  bool compressor = (this->system_outputs_ & (OUTPUT_CC | OUTPUT_CC2)) != 0;
  bool cooling = (this->system_outputs_ & OUTPUT_RV) != 0;
  bool aux_heat = (this->system_outputs_ & (OUTPUT_EH1 | OUTPUT_EH2)) != 0;
  bool blower = (this->system_outputs_ & OUTPUT_BLOWER) != 0;
  
  if (compressor) {
    if (cooling) return "Cooling";
    if (aux_heat) return "Heating + Aux";
    return "Heating";
  }
  if (aux_heat) return "Emergency Heat";
  if (blower) return "Fan Only";
  
  const uint16_t *asc = reg_find(this->register_cache_, registers::COMPRESSOR_ANTI_SHORT_CYCLE);
  if (asc != nullptr && *asc != 0) return "Waiting";
  
  return "Standby";
}

// ============================================================================
// State Machine Core
// ============================================================================

void WaterFurnaceAurora::transition_(State new_state) {
  if (this->state_ != new_state) {
    ESP_LOGV(TAG, "State: %d -> %d", static_cast<int>(this->state_), static_cast<int>(new_state));
    this->state_ = new_state;
  }
}

void WaterFurnaceAurora::send_request_(const std::vector<uint8_t> &frame, PendingRequest type,
                                        std::vector<uint16_t> expected_addrs) {
  // Clear any stale data on the bus (bounded to prevent spin if junk is streaming)
  for (int i = 0; i < 512 && this->available(); i++) {
    this->read();
  }
  this->rx_buffer_.clear();
  
  // Enable TX mode for RS485
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(true);
  }
  
  // Send frame — write_array buffers into the UART TX FIFO.
  // We intentionally do NOT call flush() here. The hardware UART transmits
  // asynchronously from its FIFO. flush() would block for up to ~106ms on a
  // 200-byte frame at 19200 baud, exceeding the 50ms WARN_IF_BLOCKING threshold.
  // Instead, the RS-485 flow control pin is switched back to RX by a deferred
  // timeout, allowing the main loop to remain non-blocking.
  this->write_array(frame.data(), frame.size());
  
  this->pending_request_ = type;
  this->expected_addresses_ = std::move(expected_addrs);
  this->last_request_time_ = millis();
  this->tx_complete_time_ = millis() + this->tx_time_ms_(frame.size());
  this->transition_(State::TX_PENDING);
}

bool WaterFurnaceAurora::read_frame_(std::vector<uint8_t> &frame) {
  // Drain available bytes into persistent rx_buffer_
  while (this->available() && this->rx_buffer_.size() < protocol::MAX_FRAME_SIZE) {
    this->rx_buffer_.push_back(this->read());
  }
  
  if (this->rx_buffer_.size() < 2) return false;
  
  // Determine expected frame size from bytes received so far
  size_t expected = protocol::expected_frame_size(this->rx_buffer_.data(), this->rx_buffer_.size());
  if (expected == 0) return false;  // Need more bytes
  if (this->rx_buffer_.size() < expected) return false;
  
  // We have enough bytes — extract the frame
  frame.assign(this->rx_buffer_.begin(), this->rx_buffer_.begin() + expected);
  this->rx_buffer_.erase(this->rx_buffer_.begin(), this->rx_buffer_.begin() + expected);
  
  // Validate CRC
  if (!protocol::validate_frame_crc(frame.data(), frame.size())) {
    ESP_LOGW(TAG, "CRC mismatch in response (%d bytes)", frame.size());
    this->status_set_warning(LOG_STR("CRC mismatch in Modbus response"));
    return false;
  }
  
  return true;
}

void WaterFurnaceAurora::process_response_(const std::vector<uint8_t> &frame) {
  // Parse frame with expected addresses
  auto resp = protocol::parse_frame(frame.data(), frame.size(), this->expected_addresses_);
  
  if (resp.is_error) {
    ESP_LOGW(TAG, "Error response (func 0x%02X, code 0x%02X) for request type %d",
             resp.function_code, resp.error_code, static_cast<int>(this->pending_request_));
    // On error during setup, go to backoff
    if (this->pending_request_ == PendingRequest::SETUP_ID ||
        this->pending_request_ == PendingRequest::SETUP_DETECT ||
        this->pending_request_ == PendingRequest::SETUP_VS_PROBE) {
      this->error_backoff_until_ = millis() + ERROR_BACKOFF_MS;
      this->transition_(State::ERROR_BACKOFF);
    } else {
      this->transition_(State::IDLE);
    }
    this->pending_request_ = PendingRequest::NONE;
    return;
  }
  
  // Successful response — update connectivity
  this->last_successful_response_ = millis();
  this->update_connected_(true);
  this->status_clear_warning();
  this->status_clear_error();
  
  // Route to appropriate handler
  switch (this->pending_request_) {
    case PendingRequest::SETUP_ID:
      this->process_setup_id_response_(resp);
      break;
    case PendingRequest::SETUP_DETECT:
      this->process_setup_detect_response_(resp);
      break;
    case PendingRequest::SETUP_VS_PROBE:
      this->process_setup_vs_probe_response_(resp);
      break;
    case PendingRequest::POLL_REGISTERS:
      this->process_poll_response_(resp);
      break;
    case PendingRequest::POLL_FAULT_HISTORY:
      this->process_fault_history_response_(resp);
      break;
    case PendingRequest::WRITE_SINGLE:
    case PendingRequest::WRITE_MULTI:
      ESP_LOGD(TAG, "Write acknowledged");
      this->transition_(State::IDLE);
      break;
    default:
      this->transition_(State::IDLE);
      break;
  }
  
  this->pending_request_ = PendingRequest::NONE;
}

void WaterFurnaceAurora::handle_timeout_() {
  ESP_LOGW(TAG, "Response timeout for request type %d (rx_buf=%d bytes)",
           static_cast<int>(this->pending_request_), this->rx_buffer_.size());
  this->rx_buffer_.clear();
  
  // During setup, use retry/backoff
  if (this->pending_request_ == PendingRequest::SETUP_ID ||
      this->pending_request_ == PendingRequest::SETUP_DETECT) {
    this->setup_retry_count_++;
    if (this->setup_retry_count_ >= MAX_SETUP_RETRIES) {
      ESP_LOGW(TAG, "Setup failed after %d retries — will continue with defaults", MAX_SETUP_RETRIES);
      // Deliberately NOT calling mark_failed() here: the heat pump may come online
      // later, so we degrade gracefully with default config rather than permanently
      // disabling the component.  status_set_error() surfaces the issue in the UI.
      this->status_set_error(LOG_STR("No response from heat pump - check RS-485 wiring"));
      this->finish_setup_();
      return;
    }
    this->error_backoff_until_ = millis() + ERROR_BACKOFF_MS;
    this->transition_(State::ERROR_BACKOFF);
  } else if (this->pending_request_ == PendingRequest::SETUP_VS_PROBE) {
    // VS probe failure just means no VS drive — continue setup
    ESP_LOGD(TAG, "VS Drive probe timed out — no VS drive");
    this->finish_setup_();
  } else {
    // Normal polling timeout
    this->status_set_warning(LOG_STR("Communication error - retrying"));
    this->transition_(State::IDLE);
  }
  
  this->pending_request_ = PendingRequest::NONE;
}

void WaterFurnaceAurora::update_connected_(bool connected) {
  if (this->connected_ == connected) return;
  this->connected_ = connected;
  if (this->connected_sensor_ != nullptr) {
    this->connected_sensor_->publish_state(connected);
  }
  if (connected) {
    ESP_LOGI(TAG, "Heat pump connected");
  } else {
    ESP_LOGW(TAG, "Heat pump disconnected (no response for %ds)", this->connected_timeout_ / 1000);
  }
}

// ============================================================================
// setup() — Non-blocking, just initializes and sets initial state
// ============================================================================

void WaterFurnaceAurora::setup() {
  ESP_LOGCONFIG(TAG, "Setting up WaterFurnace Aurora (async)...");
  
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->setup();
    this->flow_control_pin_->digital_write(false);  // Start in RX mode
  }
  
  this->rx_buffer_.reserve(protocol::MAX_FRAME_SIZE);
  this->response_frame_.reserve(protocol::MAX_FRAME_SIZE);
  this->last_successful_response_ = millis();
  this->update_connected_(false);
  
  // State machine starts at SETUP_READ_ID — loop() will drive it
  this->transition_(State::SETUP_READ_ID);

#ifdef USE_API_CUSTOM_SERVICES
  register_service(&WaterFurnaceAurora::on_write_register_service_, "write_register",
                   {"address", "value"});
  ESP_LOGD(TAG, "Registered write_register API service");
#endif

  ESP_LOGD(TAG, "Setup initialized — state machine will handle hardware detection");
}

// ============================================================================
// on_shutdown() — Ensure RS-485 transceiver is in RX mode on shutdown
// ============================================================================

void WaterFurnaceAurora::on_shutdown() {
  if (this->flow_control_pin_ != nullptr) {
    this->flow_control_pin_->digital_write(false);
  }
}

// ============================================================================
// loop() — Drives the state machine, zero blocking
// ============================================================================

void WaterFurnaceAurora::loop() {
  uint32_t now = millis();
  
  // Connectivity timeout check
  if (this->connected_ && (now - this->last_successful_response_) > this->connected_timeout_) {
    this->update_connected_(false);
  }
  
  switch (this->state_) {
    case State::SETUP_READ_ID:
      this->start_setup_read_id_();
      return;
      
    case State::SETUP_DETECT_COMPONENTS:
      this->start_setup_detect_();
      return;
      
    case State::SETUP_DETECT_VS:
      this->start_setup_vs_probe_();
      return;
      
    case State::IDLE:
      // Writes take priority over reads
      if (!this->pending_writes_.empty()) {
        this->process_pending_writes_();
        return;
      }
      // Otherwise just wait for update() to kick off a poll cycle
      return;
      
    case State::TX_PENDING:
      // Wait for UART TX FIFO to drain before switching RS-485 to RX mode.
      // This avoids calling flush() which would block for up to ~110ms on
      // large frames at 19200 baud, exceeding the 50ms loop() warning threshold.
      if (now >= this->tx_complete_time_) {
        // RS-485 turnaround: switch to RX mode now that TX is complete.
        // 500µs margin is conservative for MAX485 transceivers.
        if (this->flow_control_pin_ != nullptr) {
          this->flow_control_pin_->digital_write(false);
        }
        this->transition_(State::WAITING_RESPONSE);
      }
      return;
      
    case State::WAITING_RESPONSE: {
      // Check timeout (measured from when TX completed, not when we started transmitting)
      if ((now - this->tx_complete_time_) > RESPONSE_TIMEOUT_MS) {
        this->handle_timeout_();
        return;
      }
      
      // Try to read a complete frame (reuses pre-allocated member buffer)
      this->response_frame_.clear();
      if (this->read_frame_(this->response_frame_)) {
        this->process_response_(this->response_frame_);
      }
      return;
    }
    
    case State::ERROR_BACKOFF:
      if (now >= this->error_backoff_until_) {
        // Determine where to resume
        if (!this->setup_complete_) {
          // Retry setup — go back to whichever setup step we were on
          if (this->model_number_.empty()) {
            this->transition_(State::SETUP_READ_ID);
          } else {
            this->transition_(State::SETUP_DETECT_COMPONENTS);
          }
        } else {
          this->transition_(State::IDLE);
        }
      }
      return;
  }
}

// ============================================================================
// update() — Only kicks off a poll cycle if IDLE
// ============================================================================

void WaterFurnaceAurora::update() {
  if (!this->setup_complete_) return;  // Don't poll until setup finishes
  
  if (this->state_ == State::IDLE) {
    this->poll_tier_counter_++;
    ESP_LOGD(TAG, "Starting poll cycle %d", this->poll_tier_counter_);
    this->start_poll_cycle_();
  } else {
    ESP_LOGV(TAG, "Skipping update — state machine busy (state=%d)", static_cast<int>(this->state_));
  }
}

// ============================================================================
// dump_config()
// ============================================================================

void WaterFurnaceAurora::dump_config() {
  ESP_LOGCONFIG(TAG, "WaterFurnace Aurora:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Flow Control Pin: %s", this->flow_control_pin_ != nullptr ? "configured" : "not configured");
  ESP_LOGCONFIG(TAG, "  Read Retries: %d", this->read_retries_);
  ESP_LOGCONFIG(TAG, "  Update Interval: %dms", this->get_update_interval());
  ESP_LOGCONFIG(TAG, "  Connected Timeout: %dms", this->connected_timeout_);
  ESP_LOGCONFIG(TAG, "  Connected Sensor: %s", this->connected_sensor_ != nullptr ? "configured" : "not configured");
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

// ============================================================================
// Zone helpers
// ============================================================================

const IZ2ZoneData& WaterFurnaceAurora::get_zone_data(uint8_t zone_number) const {
  if (zone_number < 1 || zone_number > MAX_IZ2_ZONES) {
    ESP_LOGW(TAG, "Invalid zone_number %d in get_zone_data (expected 1-%d)", zone_number, MAX_IZ2_ZONES);
    return iz2_zones_[0];
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

// ============================================================================
// Setup Steps (non-blocking, driven by state machine)
// ============================================================================

void WaterFurnaceAurora::start_setup_read_id_() {
  ESP_LOGD(TAG, "Setup: reading device ID (model/serial)...");
  
  // Read model number (regs 92-103) + serial (105-109) using func 0x03 holding
  // We'll read model first; serial comes as part of detect step
  auto frame = protocol::build_read_holding_request(this->address_, registers::MODEL_NUMBER, 18);
  
  // Expected: regs 92-109 (18 registers)
  std::vector<uint16_t> expected;
  expected.reserve(18);
  for (uint16_t i = 0; i < 18; i++) {
    expected.push_back(registers::MODEL_NUMBER + i);
  }
  
  this->send_request_(frame, PendingRequest::SETUP_ID, expected);
}

void WaterFurnaceAurora::process_setup_id_response_(const protocol::ParsedResponse &resp) {
  // Extract model number (registers 92-103, 12 regs = 24 chars)
  // Stack-allocated array — no heap allocation needed.
  uint16_t model_buf[12];
  size_t model_count = 0;
  for (const auto &rv : resp.registers) {
    if (rv.address >= registers::MODEL_NUMBER && rv.address < registers::MODEL_NUMBER + 12) {
      if (model_count < 12) model_buf[model_count++] = rv.value;
    }
  }
  if (model_count > 0) {
    this->model_number_ = registers_to_string(model_buf, model_count);
    if (this->model_number_sensor_ != nullptr && !this->model_number_.empty()) {
      this->model_number_sensor_->publish_state(this->model_number_);
    }
    ESP_LOGD(TAG, "Model: '%s'", this->model_number_.c_str());
  }
  
  // Extract serial number (registers 105-109, 5 regs = 10 chars)
  uint16_t serial_buf[5];
  size_t serial_count = 0;
  for (const auto &rv : resp.registers) {
    if (rv.address >= registers::SERIAL_NUMBER && rv.address < registers::SERIAL_NUMBER + 5) {
      if (serial_count < 5) serial_buf[serial_count++] = rv.value;
    }
  }
  if (serial_count > 0) {
    this->serial_number_ = registers_to_string(serial_buf, serial_count);
    if (this->serial_number_sensor_ != nullptr && !this->serial_number_.empty()) {
      this->serial_number_sensor_->publish_state(this->serial_number_);
    }
    ESP_LOGD(TAG, "Serial: '%s'", this->serial_number_.c_str());
  }
  
  // Advance to component detection
  this->setup_retry_count_ = 0;
  this->transition_(State::SETUP_DETECT_COMPONENTS);
}

void WaterFurnaceAurora::start_setup_detect_() {
  ESP_LOGD(TAG, "Setup: detecting hardware components...");
  
  std::vector<uint16_t> addrs;
  addrs.reserve(20);
  
  if (!this->axb_override_) addrs.push_back(registers::AXB_INSTALLED);
  addrs.push_back(registers::AXB_VERSION);
  addrs.push_back(registers::THERMOSTAT_INSTALLED);
  addrs.push_back(registers::THERMOSTAT_VERSION);
  if (!this->iz2_override_) addrs.push_back(registers::IZ2_INSTALLED);
  addrs.push_back(registers::IZ2_VERSION);
  if (!this->iz2_zones_override_) addrs.push_back(registers::IZ2_NUM_ZONES);
  addrs.push_back(registers::BLOWER_TYPE);
  addrs.push_back(registers::ENERGY_MONITOR);
  addrs.push_back(registers::PUMP_TYPE);
  
  if (!this->vs_drive_override_) {
    addrs.push_back(registers::ABC_PROGRAM);      // ABC program version (4 regs)
    addrs.push_back(registers::ABC_PROGRAM + 1);
    addrs.push_back(registers::ABC_PROGRAM + 2);
    addrs.push_back(registers::ABC_PROGRAM + 3);
  }
  
  auto frame = protocol::build_read_registers_request(this->address_, addrs);
  if (frame.empty()) {
    ESP_LOGE(TAG, "Failed to build detect request");
    this->finish_setup_();
    return;
  }
  
  this->send_request_(frame, PendingRequest::SETUP_DETECT, addrs);
}

void WaterFurnaceAurora::process_setup_detect_response_(const protocol::ParsedResponse &resp) {
  // Build a RegisterMap for easy lookup
  RegisterMap result;
  result.reserve(resp.registers.size());
  for (const auto &rv : resp.registers) {
    result.emplace_back(rv.address, rv.value);
  }
  std::sort(result.begin(), result.end(),
      [](const std::pair<uint16_t, uint16_t> &a, const std::pair<uint16_t, uint16_t> &b) {
        return a.first < b.first;
      });
  
  // Detect AXB
  if (!this->axb_override_) {
    const uint16_t *val = reg_find(result, registers::AXB_INSTALLED);
    if (val) {
      this->has_axb_ = (*val != COMPONENT_NOT_INSTALLED && *val != COMPONENT_UNSUPPORTED);
      ESP_LOGD(TAG, "AXB reg %d = %d -> %s", registers::AXB_INSTALLED, *val,
               this->has_axb_ ? "present" : "absent");
    }
  }
  
  // Detect IZ2
  if (!this->iz2_override_) {
    const uint16_t *val = reg_find(result, registers::IZ2_INSTALLED);
    if (val) {
      this->has_iz2_ = (*val != COMPONENT_NOT_INSTALLED && *val != COMPONENT_UNSUPPORTED);
      ESP_LOGD(TAG, "IZ2 reg %d = %d -> %s", registers::IZ2_INSTALLED, *val,
               this->has_iz2_ ? "present" : "absent");
    }
  }
  
  // IZ2 zone count
  if (!this->iz2_zones_override_ && this->has_iz2_) {
    const uint16_t *val = reg_find(result, registers::IZ2_NUM_ZONES);
    if (val) {
      this->num_iz2_zones_ = std::min(static_cast<uint8_t>(*val), static_cast<uint8_t>(MAX_IZ2_ZONES));
      ESP_LOGD(TAG, "IZ2 zones: %d", this->num_iz2_zones_);
    }
  }
  
  // AWL versions
  const uint16_t *val_tver = reg_find(result, registers::THERMOSTAT_VERSION);
  if (val_tver) this->thermostat_version_ = static_cast<float>(*val_tver) / 100.0f;
  
  const uint16_t *val_aver = reg_find(result, registers::AXB_VERSION);
  if (val_aver) this->axb_version_ = static_cast<float>(*val_aver) / 100.0f;
  
  const uint16_t *val_iver = reg_find(result, registers::IZ2_VERSION);
  if (val_iver) this->iz2_version_ = static_cast<float>(*val_iver) / 100.0f;
  
  // Blower type
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
  }
  
  // Energy monitor
  const uint16_t *val_energy = reg_find(result, registers::ENERGY_MONITOR);
  if (val_energy) {
    this->energy_monitor_level_ = std::min(static_cast<uint8_t>(*val_energy), static_cast<uint8_t>(2));
  }
  
  // Pump type
  const uint16_t *val_pump = reg_find(result, registers::PUMP_TYPE);
  if (val_pump) {
    uint16_t pval = *val_pump;
    this->pump_type_ = (pval <= 7) ? static_cast<PumpType>(pval) : PumpType::OTHER;
  }
  
  // VS Drive detection via ABC program
  bool vs_detected_from_program = false;
  if (!this->vs_drive_override_) {
    // Stack-allocated array — no heap allocation needed (setup runs once).
    uint16_t prog_buf[4];
    size_t prog_count = 0;
    for (uint16_t r = registers::ABC_PROGRAM; r <= registers::ABC_PROGRAM + 3; r++) {
      const uint16_t *val = reg_find(result, r);
      if (val && prog_count < 4) prog_buf[prog_count++] = *val;
    }
    if (prog_count > 0) {
      std::string program = registers_to_string(prog_buf, prog_count);
      ESP_LOGD(TAG, "ABC Program: '%s'", program.c_str());
      vs_detected_from_program = (program.find("VSP") != std::string::npos ||
                                   program.find("SPLVS") != std::string::npos);
      if (vs_detected_from_program) {
        this->has_vs_drive_ = true;
      }
    }
  }
  
  // If VS not detected from program and not overridden, probe VS registers
  if (!this->vs_drive_override_ && !this->has_vs_drive_) {
    this->transition_(State::SETUP_DETECT_VS);
    return;
  }
  
  this->finish_setup_();
}

void WaterFurnaceAurora::start_setup_vs_probe_() {
  ESP_LOGD(TAG, "Setup: probing VS drive registers...");
  
  std::vector<uint16_t> addrs = {3001, 3322, 3325};
  auto frame = protocol::build_read_registers_request(this->address_, addrs);
  if (frame.empty()) {
    this->finish_setup_();
    return;
  }
  
  this->send_request_(frame, PendingRequest::SETUP_VS_PROBE, addrs);
}

void WaterFurnaceAurora::process_setup_vs_probe_response_(const protocol::ParsedResponse &resp) {
  bool has_data = false;
  for (const auto &rv : resp.registers) {
    if (rv.value != 0) {
      has_data = true;
      break;
    }
  }
  
  if (has_data) {
    this->has_vs_drive_ = true;
    ESP_LOGD(TAG, "VS Drive detected via register probe");
  } else {
    ESP_LOGD(TAG, "VS Drive not detected");
  }
  
  this->finish_setup_();
}

void WaterFurnaceAurora::finish_setup_() {
  this->setup_complete_ = true;
  
  // Publish pump type (detected once, won't change)
  if (this->pump_type_sensor_ != nullptr) {
    this->pump_type_sensor_->publish_state(get_pump_type_string(this->pump_type_));
  }
  
  ESP_LOGI(TAG, "Setup complete:");
  ESP_LOGI(TAG, "  AXB: %s%s (v%.2f)", this->has_axb_ ? "yes" : "no",
           this->axb_override_ ? " (override)" : "", this->axb_version_);
  ESP_LOGI(TAG, "  VS Drive: %s%s", this->has_vs_drive_ ? "yes" : "no",
           this->vs_drive_override_ ? " (override)" : "");
  ESP_LOGI(TAG, "  IZ2: %s%s (v%.2f, %d zones)", this->has_iz2_ ? "yes" : "no",
           this->iz2_override_ ? " (override)" : "",
           this->iz2_version_, this->num_iz2_zones_);
  ESP_LOGI(TAG, "  Blower: %s, Pump: %s, Energy: %d",
           this->blower_type_ == BlowerType::PSC ? "PSC" :
           this->blower_type_ == BlowerType::FIVE_SPEED ? "5-Speed" : "ECM",
           get_pump_type_string(this->pump_type_), this->energy_monitor_level_);
  
  // Fire deferred setup callbacks
  for (auto &cb : this->setup_callbacks_) {
    cb();
  }
  this->setup_callbacks_.clear();
  this->setup_callbacks_.shrink_to_fit();
  
  this->transition_(State::IDLE);
}

// ============================================================================
// Polling (non-blocking)
// ============================================================================

void WaterFurnaceAurora::build_poll_addresses_() {
  bool medium_poll = (this->poll_tier_counter_ % 6) == 0;
  
  this->addresses_to_read_.clear();
  if (this->addresses_to_read_.capacity() < 100) {
    this->addresses_to_read_.reserve(100);
  }
  
  // === TIER 0: Fast registers — every cycle (~5s) ===
  this->addresses_to_read_.push_back(registers::COMPRESSOR_ANTI_SHORT_CYCLE);
  this->addresses_to_read_.push_back(registers::LAST_FAULT);
  this->addresses_to_read_.push_back(registers::SYSTEM_OUTPUTS);
  this->addresses_to_read_.push_back(registers::SYSTEM_STATUS);
  
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
  
  if (this->is_ecm_blower() || this->blower_type_ == BlowerType::FIVE_SPEED) {
    this->addresses_to_read_.push_back(registers::ECM_SPEED);
  }
  
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
  
  if (this->has_axb_ && this->awl_axb()) {
    this->addresses_to_read_.push_back(registers::VS_PUMP_SPEED);
  }
  
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
    this->addresses_to_read_.push_back(registers::LAST_LOCKOUT_FAULT);
    this->addresses_to_read_.push_back(registers::OUTPUTS_AT_LOCKOUT);
    this->addresses_to_read_.push_back(registers::INPUTS_AT_LOCKOUT);
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
}

void WaterFurnaceAurora::start_poll_cycle_() {
  // Build address list based on tier
  this->build_poll_addresses_();
  
  if (this->addresses_to_read_.empty()) {
    ESP_LOGD(TAG, "No registers to poll");
    return;
  }
  
  auto frame = protocol::build_read_registers_request(this->address_, this->addresses_to_read_);
  if (frame.empty()) {
    ESP_LOGW(TAG, "Failed to build poll request (%d registers — max is %d)",
             this->addresses_to_read_.size(), protocol::MAX_REGISTERS_PER_REQUEST);
    return;
  }
  
  // Move addresses into expected_addresses_ to avoid copy (poll addresses rebuilt each cycle)
  this->send_request_(frame, PendingRequest::POLL_REGISTERS, std::move(this->addresses_to_read_));
}

void WaterFurnaceAurora::process_poll_response_(const protocol::ParsedResponse &resp) {
  // Merge response directly into cache — no intermediate allocation.
  // reg_insert() handles insert-or-update in the sorted vector.
  for (const auto &rv : resp.registers) {
    reg_insert(this->register_cache_, rv.address, rv.value);
  }
  
  // Publish all sensors from the cache
  this->publish_all_sensors_();
  
  // Check if we need fault history this cycle
  bool slow_poll = (this->poll_tier_counter_ % 60) == 0;
  if (slow_poll && this->fault_history_sensor_ != nullptr) {
    this->start_fault_history_read_();
    return;
  }
  
  this->transition_(State::IDLE);
}

void WaterFurnaceAurora::start_fault_history_read_() {
  auto frame = protocol::build_read_holding_request(this->address_, registers::FAULT_HISTORY_START, 99);
  
  // Build fault history address list once (static local, persists across calls).
  // Thread-safe: ESPHome main loop is single-threaded; no concurrent access.
  static std::vector<uint16_t> fault_history_addrs;
  if (fault_history_addrs.empty()) {
    fault_history_addrs.reserve(99);
    for (uint16_t i = 0; i < 99; i++) {
      fault_history_addrs.push_back(registers::FAULT_HISTORY_START + i);
    }
  }
  
  // Copy the static vector (send_request_ takes by value for move semantics)
  this->send_request_(frame, PendingRequest::POLL_FAULT_HISTORY,
                      std::vector<uint16_t>(fault_history_addrs));
}

void WaterFurnaceAurora::process_fault_history_response_(const protocol::ParsedResponse &resp) {
  if (this->fault_history_sensor_ == nullptr) {
    this->transition_(State::IDLE);
    return;
  }
  
  std::string history;
  history.reserve(256);
  int fault_count = 0;
  const int max_faults = 10;
  
  for (const auto &rv : resp.registers) {
    if (fault_count >= max_faults) break;
    if (rv.value == 0 || rv.value == 0xFFFF) continue;
    
    uint8_t fault_code = rv.value % 100;
    if (fault_code == 0) continue;
    
    if (!history.empty()) history += "; ";
    history += "E";
    history += std::to_string(fault_code);
    
    const char *desc = get_fault_description(fault_code);
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
  this->transition_(State::IDLE);
}

// ============================================================================
// Sensor Publishing (from cache)
// ============================================================================

void WaterFurnaceAurora::publish_all_sensors_() {
  const RegisterMap &regs = this->register_cache_;
  
  // Fault status
  {
    const uint16_t *val = reg_find(regs, registers::LAST_FAULT);
    if (val) {
      this->current_fault_ = *val & 0x7FFF;
      this->locked_out_ = (*val & 0x8000) != 0;
      if (sensor_value_changed_(this->fault_code_sensor_, this->current_fault_))
        this->fault_code_sensor_->publish_state(this->current_fault_);
      this->publish_text_if_changed(this->fault_description_sensor_, this->cached_fault_description_,
                                     get_fault_description(this->current_fault_));
      publish_binary_if_changed_(this->lockout_sensor_, this->locked_out_);
    }
  }
  
  // Lockout diagnostics (registers 26-28)
  {
    const uint16_t *val = reg_find(regs, registers::LAST_LOCKOUT_FAULT);
    if (val) {
      uint16_t lockout_code = *val & 0x7FFF;
      if (sensor_value_changed_(this->lockout_fault_sensor_, lockout_code))
        this->lockout_fault_sensor_->publish_state(lockout_code);
      this->publish_text_if_changed(this->lockout_fault_description_sensor_,
                                     this->cached_lockout_fault_description_,
                                     get_fault_description(lockout_code));
    }
  }
  {
    const uint16_t *val = reg_find(regs, registers::OUTPUTS_AT_LOCKOUT);
    if (val)
      this->publish_text_if_changed(this->outputs_at_lockout_sensor_,
                                     this->cached_outputs_at_lockout_,
                                     get_outputs_string(*val));
  }
  {
    const uint16_t *val = reg_find(regs, registers::INPUTS_AT_LOCKOUT);
    if (val)
      this->publish_text_if_changed(this->inputs_at_lockout_sensor_,
                                     this->cached_inputs_at_lockout_,
                                     get_inputs_string(*val));
  }
  
  // System outputs
  {
    const uint16_t *val = reg_find(regs, registers::SYSTEM_OUTPUTS);
    if (val) {
      this->system_outputs_ = *val;
      publish_binary_if_changed_(this->compressor_sensor_,
                                 (this->system_outputs_ & (OUTPUT_CC | OUTPUT_CC2)) != 0);
      publish_binary_if_changed_(this->blower_sensor_,
                                 (this->system_outputs_ & OUTPUT_BLOWER) != 0);
      publish_binary_if_changed_(this->aux_heat_sensor_,
                                 (this->system_outputs_ & (OUTPUT_EH1 | OUTPUT_EH2)) != 0);
      {
        float stage = static_cast<float>((this->system_outputs_ & OUTPUT_EH2) ? 2
                      : (this->system_outputs_ & OUTPUT_EH1) ? 1 : 0);
        if (sensor_value_changed_(this->aux_heat_stage_sensor_, stage))
          this->aux_heat_stage_sensor_->publish_state(stage);
      }
    }
  }
  
  // System status
  {
    const uint16_t *val = reg_find(regs, registers::SYSTEM_STATUS);
    if (val) {
      uint16_t status = *val;
      publish_binary_if_changed_(this->lps_sensor_, (status & STATUS_LPS) != 0);
      publish_binary_if_changed_(this->hps_sensor_, (status & STATUS_HPS) != 0);
      publish_binary_if_changed_(this->emergency_shutdown_sensor_,
                                 (status & STATUS_EMERGENCY_SHUTDOWN) != 0);
      publish_binary_if_changed_(this->load_shed_sensor_, (status & STATUS_LOAD_SHED) != 0);
    }
  }
  
  // AXB
  {
    const uint16_t *val = reg_find(regs, registers::AXB_INPUTS);
    if (val)
      this->publish_text_if_changed(this->axb_inputs_sensor_, this->cached_axb_inputs_,
                                     get_axb_inputs_string(*val));
  }
  {
    const uint16_t *val = reg_find(regs, registers::AXB_OUTPUTS);
    if (val) {
      this->axb_outputs_ = *val;
      publish_binary_if_changed_(this->dhw_running_sensor_,
                                 (this->axb_outputs_ & AXB_OUTPUT_DHW) != 0);
      publish_binary_if_changed_(this->loop_pump_sensor_,
                                 (this->axb_outputs_ & AXB_OUTPUT_LOOP_PUMP) != 0);
    }
  }
  
  // Active dehumidify
  {
    const uint16_t *val = reg_find(regs, registers::ACTIVE_DEHUMIDIFY);
    if (val) this->active_dehumidify_ = (*val != 0);
  }
  
  // Current mode
  this->publish_text_if_changed(this->current_mode_sensor_, this->cached_mode_string_,
                                 this->get_current_mode_string());
  
  // Temperatures
  {
    const uint16_t *val_amb = reg_find(regs, registers::AMBIENT_TEMP);
    if (val_amb) {
      this->ambient_temp_ = to_signed_tenths(*val_amb);
      if (sensor_value_changed_(this->ambient_temp_sensor_, this->ambient_temp_))
        this->ambient_temp_sensor_->publish_state(this->ambient_temp_);
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
  
  // Setpoints (respect write cooldown)
  {
    uint32_t elapsed = millis() - this->last_setpoint_write_;
    bool cooldown_active = elapsed <= WRITE_COOLDOWN_MS;
    if (cooldown_active) {
      ESP_LOGD(TAG, "Setpoint cooldown active (%ums elapsed, cached heat=%.1f cool=%.1f)",
               elapsed, this->heating_setpoint_, this->cooling_setpoint_);
    } else {
      const uint16_t *val_hsp = reg_find(regs, registers::HEATING_SETPOINT);
      if (val_hsp) {
        float new_val = to_tenths(*val_hsp);
        if (new_val != this->heating_setpoint_) {
          ESP_LOGD(TAG, "Heating SP from device: %.1f -> %.1f", this->heating_setpoint_, new_val);
        }
        this->heating_setpoint_ = new_val;
        if (sensor_value_changed_(this->heating_setpoint_sensor_, this->heating_setpoint_))
          this->heating_setpoint_sensor_->publish_state(this->heating_setpoint_);
      }
      const uint16_t *val_csp = reg_find(regs, registers::COOLING_SETPOINT);
      if (val_csp) {
        float new_val = to_tenths(*val_csp);
        if (new_val != this->cooling_setpoint_) {
          ESP_LOGD(TAG, "Cooling SP from device: %.1f -> %.1f", this->cooling_setpoint_, new_val);
        }
        this->cooling_setpoint_ = new_val;
        if (sensor_value_changed_(this->cooling_setpoint_sensor_, this->cooling_setpoint_))
          this->cooling_setpoint_sensor_->publish_state(this->cooling_setpoint_);
      }
    }
  }
  
  // DHW
  {
    const uint16_t *val_dhw = reg_find(regs, registers::DHW_ENABLED);
    if (val_dhw) this->dhw_enabled_ = (*val_dhw != 0);
  }
  {
    const uint16_t *val_dhwsp = reg_find(regs, registers::DHW_SETPOINT);
    if (val_dhwsp) {
      this->dhw_setpoint_ = to_tenths(*val_dhwsp);
      if (sensor_value_changed_(this->dhw_setpoint_sensor_, this->dhw_setpoint_))
        this->dhw_setpoint_sensor_->publish_state(this->dhw_setpoint_);
    }
  }
  {
    const uint16_t *val_dhwt = reg_find(regs, registers::DHW_TEMP);
    if (val_dhwt) {
      this->dhw_temp_ = to_signed_tenths(*val_dhwt);
      if (sensor_value_changed_(this->dhw_temp_sensor_, this->dhw_temp_))
        this->dhw_temp_sensor_->publish_state(this->dhw_temp_);
    }
  }
  
  // HVAC mode (respect write cooldown)
  if ((millis() - this->last_mode_write_) > WRITE_COOLDOWN_MS) {
    const uint16_t *val_mode = reg_find(regs, registers::HEATING_MODE_READ);
    if (val_mode) {
      uint8_t mode_val = (*val_mode >> 8) & 0x07;
      this->hvac_mode_ = static_cast<HeatingMode>(mode_val);
      this->publish_text_if_changed(this->hvac_mode_sensor_, this->cached_hvac_mode_,
                                     get_hvac_mode_string(this->hvac_mode_));
    }
  }
  
  // Fan mode (respect write cooldown)
  if ((millis() - this->last_fan_write_) > WRITE_COOLDOWN_MS) {
    const uint16_t *val_fan = reg_find(regs, registers::FAN_CONFIG);
    if (val_fan) {
      uint16_t config = *val_fan;
      if (config & 0x80)
        this->fan_mode_ = FanMode::CONTINUOUS;
      else if (config & 0x100)
        this->fan_mode_ = FanMode::INTERMITTENT;
      else
        this->fan_mode_ = FanMode::AUTO;
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
    if (val_lp && *val_lp < 10000) {
      float fval = to_tenths(*val_lp);
      if (sensor_value_changed_(this->loop_pressure_sensor_, fval))
        this->loop_pressure_sensor_->publish_state(fval);
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
  
  // VS Drive status strings
  {
    const uint16_t *val = reg_find(regs, registers::VS_DERATE);
    if (val) this->publish_text_if_changed(this->vs_derate_sensor_, this->cached_vs_derate_,
                                            get_vs_derate_string(*val));
  }
  {
    const uint16_t *val = reg_find(regs, registers::VS_SAFE_MODE);
    if (val) this->publish_text_if_changed(this->vs_safe_mode_sensor_, this->cached_vs_safe_mode_,
                                            get_vs_safe_mode_string(*val));
  }
  {
    const uint16_t *val_a1 = reg_find(regs, registers::VS_ALARM1);
    const uint16_t *val_a2 = reg_find(regs, registers::VS_ALARM2);
    if (val_a1 && val_a2) this->publish_text_if_changed(this->vs_alarm_sensor_, this->cached_vs_alarm_,
                                                          get_vs_alarm_string(*val_a1, *val_a2));
  }
  
  // FP1/FP2
  this->publish_sensor_signed_tenths(regs, registers::FP1_TEMP, this->fp1_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::FP2_TEMP, this->fp2_sensor_);
  
  // Line voltage setting and anti-short-cycle
  this->publish_sensor(regs, registers::LINE_VOLTAGE_SETTING, this->line_voltage_setting_sensor_);
  this->publish_sensor(regs, registers::COMPRESSOR_ANTI_SHORT_CYCLE, this->anti_short_cycle_sensor_);
  
  // Blower/ECM
  this->publish_sensor(regs, registers::ECM_SPEED, this->blower_speed_sensor_);
  this->publish_sensor(regs, registers::BLOWER_ONLY_SPEED, this->blower_only_speed_sensor_);
  this->publish_sensor(regs, registers::LO_COMPRESSOR_ECM_SPEED, this->lo_compressor_speed_sensor_);
  this->publish_sensor(regs, registers::HI_COMPRESSOR_ECM_SPEED, this->hi_compressor_speed_sensor_);
  this->publish_sensor(regs, registers::AUX_HEAT_ECM_SPEED, this->aux_heat_speed_sensor_);
  
  // VS Pump
  this->publish_sensor(regs, registers::VS_PUMP_SPEED, this->pump_speed_sensor_);
  this->publish_sensor(regs, registers::VS_PUMP_MIN, this->pump_min_speed_sensor_);
  this->publish_sensor(regs, registers::VS_PUMP_MAX, this->pump_max_speed_sensor_);
  
  // Refrigeration
  this->publish_sensor_signed_tenths(regs, registers::HEATING_LIQUID_LINE_TEMP, this->heating_liquid_line_temp_sensor_);
  this->publish_sensor_signed_tenths(regs, registers::SATURATED_CONDENSER_TEMP, this->saturated_condenser_temp_sensor_);
  
  bool is_cooling = (this->system_outputs_ & OUTPUT_RV) != 0;
  uint16_t subcool_reg = is_cooling ? registers::SUBCOOL_COOLING : registers::SUBCOOL_HEATING;
  {
    const uint16_t *val = reg_find(regs, subcool_reg);
    if (val) {
      float fval = to_signed_tenths(*val);
      if (sensor_value_changed_(this->subcool_temp_sensor_, fval))
        this->subcool_temp_sensor_->publish_state(fval);
    }
  }
  
  this->publish_sensor_int32(regs, registers::HEAT_OF_EXTRACTION, this->heat_of_extraction_sensor_);
  this->publish_sensor_int32(regs, registers::HEAT_OF_REJECTION, this->heat_of_rejection_sensor_);
  
  // Humidifier/Dehumidifier running
  publish_binary_if_changed_(this->humidifier_running_sensor_,
                             (this->system_outputs_ & OUTPUT_ACCESSORY) != 0);
  publish_binary_if_changed_(this->dehumidifier_running_sensor_,
                             this->active_dehumidify_ || ((this->axb_outputs_ & 0x10) != 0));
  
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
      float hum_target = static_cast<float>((*val_targets >> 8) & 0xFF);
      if (sensor_value_changed_(this->humidification_target_sensor_, hum_target))
        this->humidification_target_sensor_->publish_state(hum_target);
      float dehum_target = static_cast<float>(*val_targets & 0xFF);
      if (sensor_value_changed_(this->dehumidification_target_sensor_, dehum_target))
        this->dehumidification_target_sensor_->publish_state(dehum_target);
    }
  }
  
  // IZ2 zone data
  if (this->has_iz2_ && this->num_iz2_zones_ > 0) {
    for (uint8_t zone = 1; zone <= this->num_iz2_zones_; zone++) {
      IZ2ZoneData &zone_data = this->iz2_zones_[zone - 1];
      
      const uint16_t *val_amb = reg_find(regs, registers::IZ2_AMBIENT_BASE + ((zone - 1) * 3));
      if (val_amb) zone_data.ambient_temperature = to_signed_tenths(*val_amb);
      
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
    }
    
    this->publish_sensor(regs, registers::IZ2_COMPRESSOR_SPEED_DESIRED, this->iz2_compressor_speed_sensor_);
    {
      const uint16_t *val = reg_find(regs, registers::IZ2_BLOWER_SPEED_DESIRED);
      if (val && this->iz2_blower_speed_sensor_ != nullptr)
        this->iz2_blower_speed_sensor_->publish_state(iz2_fan_desired(*val));
    }
  }
  
  // Derived sensors
  this->publish_derived_sensors(regs);
  
  // Notify listeners
  for (auto &listener : this->listeners_) {
    listener();
  }
}

// ============================================================================
// Write Queue
// ============================================================================

void WaterFurnaceAurora::write_register(uint16_t addr, uint16_t value) {
  // Check if there's already a pending write for this address and update it
  for (auto &pw : this->pending_writes_) {
    if (pw.first == addr) {
      pw.second = value;
      return;
    }
  }
  if (this->pending_writes_.size() >= MAX_PENDING_WRITES) {
    ESP_LOGW(TAG, "Write queue full (%d), dropping write to reg %d", MAX_PENDING_WRITES, addr);
    return;
  }
  this->pending_writes_.emplace_back(addr, value);
}

void WaterFurnaceAurora::process_pending_writes_() {
  if (this->pending_writes_.empty()) return;
  
  if (this->pending_writes_.size() == 1) {
    // Single write — use func 0x06
    auto &w = this->pending_writes_[0];
    ESP_LOGD(TAG, "Writing register %d = %d", w.first, w.second);
    auto frame = protocol::build_write_single_request(this->address_, w.first, w.second);
    this->pending_writes_.clear();
    this->send_request_(frame, PendingRequest::WRITE_SINGLE);
  } else {
    // Batch write — use func 0x43
    ESP_LOGD(TAG, "Batch writing %d registers", this->pending_writes_.size());
    auto frame = protocol::build_write_multi_request(this->address_, this->pending_writes_);
    this->pending_writes_.clear();
    if (frame.empty()) {
      ESP_LOGW(TAG, "Failed to build batch write request");
      return;
    }
    this->send_request_(frame, PendingRequest::WRITE_MULTI);
  }
}

// ============================================================================
// Control Methods (queue writes, return true for success)
// ============================================================================

bool WaterFurnaceAurora::set_heating_setpoint(float temp) {
  if (temp < 40.0f || temp > 90.0f) {
    ESP_LOGW(TAG, "Heating setpoint %.1f out of range (40-90)", temp);
    return false;
  }
  uint16_t value = static_cast<uint16_t>(temp * 10);
  this->write_register(registers::HEATING_SETPOINT_WRITE, value);
  this->heating_setpoint_ = temp;  // Optimistic update — prevents stale read-back during cooldown
  this->last_setpoint_write_ = millis();
  return true;
}

bool WaterFurnaceAurora::set_cooling_setpoint(float temp) {
  if (temp < 54.0f || temp > 99.0f) {
    ESP_LOGW(TAG, "Cooling setpoint %.1f out of range (54-99)", temp);
    return false;
  }
  uint16_t value = static_cast<uint16_t>(temp * 10);
  this->write_register(registers::COOLING_SETPOINT_WRITE, value);
  this->cooling_setpoint_ = temp;  // Optimistic update — prevents stale read-back during cooldown
  this->last_setpoint_write_ = millis();
  return true;
}

bool WaterFurnaceAurora::set_hvac_mode(HeatingMode mode) {
  this->write_register(registers::HEATING_MODE_WRITE, static_cast<uint16_t>(mode));
  this->hvac_mode_ = mode;  // Optimistic update — prevents stale read-back during cooldown
  this->last_mode_write_ = millis();
  return true;
}

bool WaterFurnaceAurora::set_fan_mode(FanMode mode) {
  this->write_register(registers::FAN_MODE_WRITE, static_cast<uint16_t>(mode));
  this->fan_mode_ = mode;  // Optimistic update — prevents stale read-back during cooldown
  this->last_fan_write_ = millis();
  return true;
}

bool WaterFurnaceAurora::set_dhw_enabled(bool enabled) {
  this->write_register(registers::DHW_ENABLED, enabled ? 1 : 0);
  this->dhw_enabled_ = enabled;  // Optimistic update
  return true;
}

bool WaterFurnaceAurora::set_dhw_setpoint(float temp) {
  if (temp < 100.0f || temp > 140.0f) {
    ESP_LOGW(TAG, "DHW setpoint %.1f out of range (100-140)", temp);
    return false;
  }
  this->write_register(registers::DHW_SETPOINT, static_cast<uint16_t>(temp * 10));
  this->dhw_setpoint_ = temp;  // Optimistic update
  return true;
}

bool WaterFurnaceAurora::set_blower_only_speed(uint8_t speed) {
  if (speed < 1 || speed > 12) { ESP_LOGW(TAG, "Blower speed %d out of range", speed); return false; }
  this->write_register(registers::BLOWER_ONLY_SPEED, speed);
  return true;
}

bool WaterFurnaceAurora::set_lo_compressor_speed(uint8_t speed) {
  if (speed < 1 || speed > 12) { ESP_LOGW(TAG, "Lo compressor speed %d out of range", speed); return false; }
  this->write_register(registers::LO_COMPRESSOR_ECM_SPEED, speed);
  return true;
}

bool WaterFurnaceAurora::set_hi_compressor_speed(uint8_t speed) {
  if (speed < 1 || speed > 12) { ESP_LOGW(TAG, "Hi compressor speed %d out of range", speed); return false; }
  this->write_register(registers::HI_COMPRESSOR_ECM_SPEED, speed);
  return true;
}

bool WaterFurnaceAurora::set_aux_heat_ecm_speed(uint8_t speed) {
  if (speed < 1 || speed > 12) { ESP_LOGW(TAG, "Aux heat speed %d out of range", speed); return false; }
  this->write_register(registers::AUX_HEAT_ECM_SPEED, speed);
  return true;
}

bool WaterFurnaceAurora::set_pump_speed(uint8_t speed) {
  if (speed < 1 || speed > 100) { ESP_LOGW(TAG, "Pump speed %d out of range", speed); return false; }
  this->write_register(registers::VS_PUMP_MANUAL, speed);
  return true;
}

bool WaterFurnaceAurora::set_pump_min_speed(uint8_t speed) {
  if (speed < 1 || speed > 100) { ESP_LOGW(TAG, "Pump min speed %d out of range", speed); return false; }
  this->write_register(registers::VS_PUMP_MIN, speed);
  return true;
}

bool WaterFurnaceAurora::set_pump_max_speed(uint8_t speed) {
  if (speed < 1 || speed > 100) { ESP_LOGW(TAG, "Pump max speed %d out of range", speed); return false; }
  this->write_register(registers::VS_PUMP_MAX, speed);
  return true;
}

bool WaterFurnaceAurora::set_fan_intermittent_on(uint8_t minutes) {
  if (minutes > 25 || (minutes % 5) != 0) {
    ESP_LOGW(TAG, "Fan on time %d invalid", minutes);
    return false;
  }
  this->write_register(registers::FAN_INTERMITTENT_ON_WRITE, minutes);
  return true;
}

bool WaterFurnaceAurora::set_fan_intermittent_off(uint8_t minutes) {
  if (minutes < 5 || minutes > 40 || (minutes % 5) != 0) {
    ESP_LOGW(TAG, "Fan off time %d invalid", minutes);
    return false;
  }
  this->write_register(registers::FAN_INTERMITTENT_OFF_WRITE, minutes);
  return true;
}

bool WaterFurnaceAurora::set_humidification_target(uint8_t percent) {
  if (percent < 15 || percent > 50) {
    ESP_LOGW(TAG, "Humidification target %d out of range (15-50)", percent);
    return false;
  }
  // Need current dehumidification target to build the combined register value
  uint16_t target_reg = (this->has_iz2_ && this->awl_communicating())
                            ? registers::IZ2_HUMIDISTAT_TARGETS
                            : registers::HUMIDISTAT_TARGETS;
  uint16_t write_reg = (this->has_iz2_ && this->awl_communicating())
                           ? registers::IZ2_HUMIDISTAT_TARGETS_WRITE
                           : registers::HUMIDISTAT_TARGETS;
  const uint16_t *current = reg_find(this->register_cache_, target_reg);
  uint8_t dehum_target = current ? (*current & 0xFF) : 50;
  this->write_register(write_reg, (percent << 8) | dehum_target);
  return true;
}

bool WaterFurnaceAurora::set_dehumidification_target(uint8_t percent) {
  if (percent < 35 || percent > 65) {
    ESP_LOGW(TAG, "Dehumidification target %d out of range (35-65)", percent);
    return false;
  }
  uint16_t target_reg = (this->has_iz2_ && this->awl_communicating())
                            ? registers::IZ2_HUMIDISTAT_TARGETS
                            : registers::HUMIDISTAT_TARGETS;
  uint16_t write_reg = (this->has_iz2_ && this->awl_communicating())
                           ? registers::IZ2_HUMIDISTAT_TARGETS_WRITE
                           : registers::HUMIDISTAT_TARGETS;
  const uint16_t *current = reg_find(this->register_cache_, target_reg);
  uint8_t hum_target = current ? ((*current >> 8) & 0xFF) : 35;
  this->write_register(write_reg, (hum_target << 8) | percent);
  return true;
}

bool WaterFurnaceAurora::clear_fault_history() {
  ESP_LOGI(TAG, "Clearing fault history");
  this->write_register(registers::CLEAR_FAULT_HISTORY, registers::CLEAR_FAULT_MAGIC);
  return true;
}

// IZ2 Zone controls
bool WaterFurnaceAurora::set_zone_heating_setpoint(uint8_t zone_number, float temp) {
  if (!this->validate_zone_number(zone_number)) return false;
  if (temp < 40.0f || temp > 90.0f) { ESP_LOGW(TAG, "Zone heating SP out of range"); return false; }
  uint16_t reg = registers::IZ2_HEAT_SP_WRITE_BASE + ((zone_number - 1) * 9);
  this->write_register(reg, static_cast<uint16_t>(temp * 10));
  this->last_setpoint_write_ = millis();
  return true;
}

bool WaterFurnaceAurora::set_zone_cooling_setpoint(uint8_t zone_number, float temp) {
  if (!this->validate_zone_number(zone_number)) return false;
  if (temp < 54.0f || temp > 99.0f) { ESP_LOGW(TAG, "Zone cooling SP out of range"); return false; }
  uint16_t reg = registers::IZ2_COOL_SP_WRITE_BASE + ((zone_number - 1) * 9);
  this->write_register(reg, static_cast<uint16_t>(temp * 10));
  this->last_setpoint_write_ = millis();
  return true;
}

bool WaterFurnaceAurora::set_zone_hvac_mode(uint8_t zone_number, HeatingMode mode) {
  if (!this->validate_zone_number(zone_number)) return false;
  uint16_t reg = registers::IZ2_MODE_WRITE_BASE + ((zone_number - 1) * 9);
  this->write_register(reg, static_cast<uint16_t>(mode));
  this->last_mode_write_ = millis();
  return true;
}

bool WaterFurnaceAurora::set_zone_fan_mode(uint8_t zone_number, FanMode mode) {
  if (!this->validate_zone_number(zone_number)) return false;
  uint16_t reg = registers::IZ2_FAN_MODE_WRITE_BASE + ((zone_number - 1) * 9);
  this->write_register(reg, static_cast<uint16_t>(mode));
  this->last_fan_write_ = millis();
  return true;
}

bool WaterFurnaceAurora::set_zone_fan_intermittent_on(uint8_t zone_number, uint8_t minutes) {
  if (!this->validate_zone_number(zone_number)) return false;
  if (minutes > 25 || (minutes % 5) != 0) { ESP_LOGW(TAG, "Fan on time invalid"); return false; }
  uint16_t reg = registers::IZ2_FAN_ON_WRITE_BASE + ((zone_number - 1) * 9);
  this->write_register(reg, minutes);
  return true;
}

bool WaterFurnaceAurora::set_zone_fan_intermittent_off(uint8_t zone_number, uint8_t minutes) {
  if (!this->validate_zone_number(zone_number)) return false;
  if (minutes < 5 || minutes > 40 || (minutes % 5) != 0) { ESP_LOGW(TAG, "Fan off time invalid"); return false; }
  uint16_t reg = registers::IZ2_FAN_OFF_WRITE_BASE + ((zone_number - 1) * 9);
  this->write_register(reg, minutes);
  return true;
}

// ============================================================================
// Derived Sensors
// ============================================================================

constexpr float BTU_PER_WATT_HOUR = 3.412f;

void WaterFurnaceAurora::publish_derived_sensors(const RegisterMap &regs) {
  // Water delta-T
  if (this->water_delta_t_sensor_ != nullptr) {
    const uint16_t *val_lw = reg_find(regs, registers::LEAVING_WATER);
    const uint16_t *val_ew = reg_find(regs, registers::ENTERING_WATER);
    if (val_lw && val_ew) {
      this->water_delta_t_sensor_->publish_state(to_signed_tenths(*val_lw) - to_signed_tenths(*val_ew));
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
          bool cooling = (this->system_outputs_ & OUTPUT_RV) != 0;
          uint16_t heat_reg = cooling ? registers::HEAT_OF_EXTRACTION : registers::HEAT_OF_REJECTION;
          const uint16_t *heat_h = reg_find(regs, heat_reg);
          const uint16_t *heat_l = reg_find(regs, heat_reg + 1);
          if (heat_h && heat_l) {
            int32_t heat_btu = to_int32(*heat_h, *heat_l);
            float cop = static_cast<float>(std::abs(heat_btu)) /
                        (static_cast<float>(total_watts) * BTU_PER_WATT_HOUR);
            if (cop >= 0.5f && cop <= 15.0f)
              this->cop_sensor_->publish_state(cop);
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
      bool cooling = (this->system_outputs_ & OUTPUT_RV) != 0;
      if (cooling && val_ew)
        this->approach_temp_sensor_->publish_state(sat_cond - to_signed_tenths(*val_ew));
      else if (!cooling && val_lw)
        this->approach_temp_sensor_->publish_state(to_signed_tenths(*val_lw) - sat_cond);
    }
  }
}

// ============================================================================
// HA API Custom Services
// ============================================================================

#ifdef USE_API_CUSTOM_SERVICES

void WaterFurnaceAurora::on_write_register_service_(int32_t address, int32_t value) {
  if (address < 0 || address > 65535 || value < 0 || value > 65535) {
    ESP_LOGW(TAG, "API write_register: invalid args address=%d value=%d",
             static_cast<int>(address), static_cast<int>(value));
    return;
  }
  ESP_LOGI(TAG, "API write_register: address=%d value=%d",
           static_cast<int>(address), static_cast<int>(value));
  this->write_register(static_cast<uint16_t>(address),
                       static_cast<uint16_t>(value));
}

#endif  // USE_API_CUSTOM_SERVICES

}  // namespace waterfurnace_aurora
}  // namespace esphome
