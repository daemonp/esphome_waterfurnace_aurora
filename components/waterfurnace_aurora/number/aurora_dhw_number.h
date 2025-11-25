#pragma once

#include "esphome/core/component.h"
#include "esphome/components/number/number.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

class AuroraDHWNumber : public number::Number, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }

 protected:
  void control(float value) override;

  WaterFurnaceAurora *parent_{nullptr};
  uint32_t last_update_{0};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
