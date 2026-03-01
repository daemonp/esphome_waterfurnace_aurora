#pragma once

#include "esphome/core/component.h"
#include "esphome/components/select/select.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

enum class AuroraHumidistatType : uint8_t {
  HUMIDIFIER,
  DEHUMIDIFIER,
};

class AuroraHumidistatSelect : public select::Select, public Component {
 public:
  void setup() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }
  void set_type(AuroraHumidistatType type) { this->type_ = type; }

 protected:
  void control(const std::string &value) override;
  void update_state_();

  WaterFurnaceAurora *parent_{nullptr};
  AuroraHumidistatType type_{AuroraHumidistatType::HUMIDIFIER};
  bool has_published_{false};
  bool last_auto_{false};
};

/// Select entity for manual operation mode (commissioning).
/// Options: Off, Heating, Cooling, Heating+Aux
/// When switched from Off to a mode, writes register 3002 with the configured
/// compressor/blower speeds. When switched to Off, writes 0x7FFF.
class AuroraManualOperationSelect : public select::Select, public Component {
 public:
  void setup() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }
  void set_compressor_speed(uint8_t speed) { this->compressor_speed_ = speed; }
  void set_blower_speed(uint8_t speed) { this->blower_speed_ = speed; }

 protected:
  void control(const std::string &value) override;

  WaterFurnaceAurora *parent_{nullptr};
  uint8_t compressor_speed_{8};   ///< Default compressor speed for manual mode
  uint8_t blower_speed_{255};     ///< 255 = "with_compressor" (auto), 0-15 = manual
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
