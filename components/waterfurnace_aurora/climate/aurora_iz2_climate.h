#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

class AuroraIZ2Climate : public climate::Climate, public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }
  void set_zone_number(uint8_t zone) { this->zone_number_ = zone; }

  // Climate traits
  climate::ClimateTraits traits() override;

  // Climate control
  void control(const climate::ClimateCall &call) override;

 protected:
  void update_state_();

  WaterFurnaceAurora *parent_{nullptr};
  uint8_t zone_number_{1};
  uint32_t last_update_{0};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
