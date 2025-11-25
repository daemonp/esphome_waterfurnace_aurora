#pragma once

#include "esphome/core/component.h"
#include "esphome/components/button/button.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

class AuroraClearFaultButton : public button::Button, public Component {
 public:
  void setup() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }

 protected:
  void press_action() override;

  WaterFurnaceAurora *parent_{nullptr};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
