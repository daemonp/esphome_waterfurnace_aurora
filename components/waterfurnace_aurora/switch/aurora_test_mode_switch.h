#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

/// Switch entity for test mode (register 45).
/// Write-only: 1 = enable, 0 = disable. No read-back register.
class AuroraTestModeSwitch : public switch_::Switch, public Component {
 public:
  void setup() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;

  WaterFurnaceAurora *parent_{nullptr};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
