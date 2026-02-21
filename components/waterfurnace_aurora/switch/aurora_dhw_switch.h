#pragma once

#include "esphome/core/component.h"
#include "esphome/components/switch/switch.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

class AuroraDHWSwitch : public switch_::Switch, public Component {
 public:
  void loop() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }

 protected:
  void write_state(bool state) override;

  WaterFurnaceAurora *parent_{nullptr};
  uint32_t last_update_{0};
  bool last_state_{false};
  bool has_published_{false};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
