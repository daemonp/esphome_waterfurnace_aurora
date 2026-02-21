#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

class AuroraDHWSwitch : public switch_::Switch, public Component {
 public:
  void setup() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;
  void update_state_();

  WaterFurnaceAurora *parent_{nullptr};
  bool last_state_{false};
  bool has_published_{false};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
