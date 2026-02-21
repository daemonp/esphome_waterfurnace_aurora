#include "aurora_dhw_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace waterfurnace_aurora {

static const char *const TAG = "aurora.number";

void AuroraDHWNumber::setup() {
  // Register with parent to receive data update notifications.
  if (this->parent_ != nullptr) {
    this->parent_->register_listener([this]() { this->update_state_(); });
  }
}

void AuroraDHWNumber::update_state_() {
  if (this->parent_ == nullptr) return;
  float setpoint = this->parent_->get_dhw_setpoint();
  if (!std::isnan(setpoint) && setpoint != this->last_value_) {
    this->publish_state(setpoint);
    this->last_value_ = setpoint;
  }
}

void AuroraDHWNumber::dump_config() {
  ESP_LOGCONFIG(TAG, "Aurora DHW Setpoint Number:");
  ESP_LOGCONFIG(TAG, "  Parent: %s", this->parent_ != nullptr ? "configured" : "NOT SET");
}

void AuroraDHWNumber::control(float value) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control DHW setpoint");
    return;
  }
  
  if (this->parent_->set_dhw_setpoint(value)) {
    this->publish_state(value);
  }
}

// Generic AuroraNumber implementation — no listener needed, state is write-only
void AuroraNumber::dump_config() {
  const char *type_name = "Unknown";
  switch (this->type_) {
    case AuroraNumberType::BLOWER_ONLY_SPEED: type_name = "Blower Only Speed"; break;
    case AuroraNumberType::LO_COMPRESSOR_SPEED: type_name = "Lo Compressor Speed"; break;
    case AuroraNumberType::HI_COMPRESSOR_SPEED: type_name = "Hi Compressor Speed"; break;
    case AuroraNumberType::AUX_HEAT_SPEED: type_name = "Aux Heat Speed"; break;
    case AuroraNumberType::PUMP_SPEED: type_name = "Pump Speed"; break;
    case AuroraNumberType::PUMP_MIN_SPEED: type_name = "Pump Min Speed"; break;
    case AuroraNumberType::PUMP_MAX_SPEED: type_name = "Pump Max Speed"; break;
    case AuroraNumberType::FAN_INTERMITTENT_ON: type_name = "Fan Intermittent On"; break;
    case AuroraNumberType::FAN_INTERMITTENT_OFF: type_name = "Fan Intermittent Off"; break;
    case AuroraNumberType::HUMIDIFICATION_TARGET: type_name = "Humidification Target"; break;
    case AuroraNumberType::DEHUMIDIFICATION_TARGET: type_name = "Dehumidification Target"; break;
    default: break;
  }
  ESP_LOGCONFIG(TAG, "Aurora Number: %s", type_name);
  ESP_LOGCONFIG(TAG, "  Parent: %s", this->parent_ != nullptr ? "configured" : "NOT SET");
}

void AuroraNumber::control(float value) {
  if (this->parent_ == nullptr) {
    ESP_LOGW(TAG, "Parent not set, cannot control");
    return;
  }
  
  bool success = false;
  uint8_t int_value = static_cast<uint8_t>(value);
  
  switch (this->type_) {
    case AuroraNumberType::BLOWER_ONLY_SPEED:
      success = this->parent_->set_blower_only_speed(int_value);
      break;
    case AuroraNumberType::LO_COMPRESSOR_SPEED:
      success = this->parent_->set_lo_compressor_speed(int_value);
      break;
    case AuroraNumberType::HI_COMPRESSOR_SPEED:
      success = this->parent_->set_hi_compressor_speed(int_value);
      break;
    case AuroraNumberType::AUX_HEAT_SPEED:
      success = this->parent_->set_aux_heat_ecm_speed(int_value);
      break;
    case AuroraNumberType::PUMP_SPEED:
      success = this->parent_->set_pump_speed(int_value);
      break;
    case AuroraNumberType::PUMP_MIN_SPEED:
      success = this->parent_->set_pump_min_speed(int_value);
      break;
    case AuroraNumberType::PUMP_MAX_SPEED:
      success = this->parent_->set_pump_max_speed(int_value);
      break;
    case AuroraNumberType::FAN_INTERMITTENT_ON:
      success = this->parent_->set_fan_intermittent_on(int_value);
      break;
    case AuroraNumberType::FAN_INTERMITTENT_OFF:
      success = this->parent_->set_fan_intermittent_off(int_value);
      break;
    case AuroraNumberType::HUMIDIFICATION_TARGET:
      success = this->parent_->set_humidification_target(int_value);
      break;
    case AuroraNumberType::DEHUMIDIFICATION_TARGET:
      success = this->parent_->set_dehumidification_target(int_value);
      break;
    default:
      ESP_LOGW(TAG, "Unknown number type");
      return;
  }
  
  if (success) {
    this->publish_state(value);
  }
}

}  // namespace waterfurnace_aurora
}  // namespace esphome
