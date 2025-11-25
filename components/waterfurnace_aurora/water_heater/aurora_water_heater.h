#pragma once

#include "esphome/core/component.h"
#include "esphome/components/water_heater/water_heater.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

class AuroraWaterHeater : public water_heater::WaterHeater, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }

  // Water heater traits
  water_heater::WaterHeaterTraits traits() override;

  // Water heater control
  void control(const water_heater::WaterHeaterCall &call) override;

 protected:
  void update_state_();

  WaterFurnaceAurora *parent_{nullptr};
  uint32_t last_update_{0};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
