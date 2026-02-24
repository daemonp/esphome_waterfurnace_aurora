#pragma once

#include "esphome/core/component.h"
#include "esphome/core/version.h"
#include "esphome/components/water_heater/water_heater.h"
#include "../waterfurnace_aurora.h"

namespace esphome {
namespace waterfurnace_aurora {

// In ESPHome 2026.2+ WaterHeater no longer inherits Component, so we add it.
// In ESPHome <= 2026.1.x WaterHeater already inherits Component; the
// preprocessor guard below avoids a diamond-inheritance ambiguity.
#if defined(ESPHOME_VERSION_CODE) && ESPHOME_VERSION_CODE >= VERSION_CODE(2026, 2, 0)
class AuroraWaterHeater : public water_heater::WaterHeater, public Component {
#else
class AuroraWaterHeater : public water_heater::WaterHeater {
#endif
 public:
  void setup() override;
  void dump_config() override;

  float get_setup_priority() const override { return setup_priority::PROCESSOR; }

  void set_parent(WaterFurnaceAurora *parent) { this->parent_ = parent; }

  // WaterHeater interface
  water_heater::WaterHeaterTraits traits() override;
  water_heater::WaterHeaterCallInternal make_call() override;
  void control(const water_heater::WaterHeaterCall &call) override;

 protected:
  void update_state_();

  WaterFurnaceAurora *parent_{nullptr};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
