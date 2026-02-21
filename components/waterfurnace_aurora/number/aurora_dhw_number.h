#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

// Control type enum for generic number entities
enum class AuroraNumberType : uint8_t {
  DHW_SETPOINT,
  BLOWER_ONLY_SPEED,
  LO_COMPRESSOR_SPEED,
  HI_COMPRESSOR_SPEED,
  AUX_HEAT_SPEED,
  PUMP_SPEED,
  PUMP_MIN_SPEED,
  PUMP_MAX_SPEED,
  FAN_INTERMITTENT_ON,
  FAN_INTERMITTENT_OFF,
  HUMIDIFICATION_TARGET,
  DEHUMIDIFICATION_TARGET,
};

class AuroraDHWNumber : public number::Number, public Component {
 public:
  void setup() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }

 protected:
  void control(float value) override;
  void update_state_();

  WaterFurnaceAurora *parent_{nullptr};
  float last_value_{NAN};
};

// Generic number class for all Aurora controls.
// Registers a listener on the hub to receive read-back values from the register cache.
class AuroraNumber : public number::Number, public Component {
 public:
  void setup() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }
  void set_type(AuroraNumberType type) { this->type_ = type; }

 protected:
  void control(float value) override;
  void update_state_();

  WaterFurnaceAurora *parent_{nullptr};
  AuroraNumberType type_{AuroraNumberType::DHW_SETPOINT};
  float last_value_{NAN};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
