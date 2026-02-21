#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

class AuroraClimate : public climate::Climate, public Component {
 public:
  void setup() override;
  void dump_config() override;
  
  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }

  // Climate traits
  climate::ClimateTraits traits() override;

  // Climate control
  void control(const climate::ClimateCall &call) override;

 protected:
  void update_state_();

  WaterFurnaceAurora *parent_{nullptr};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
