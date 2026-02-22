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

}  // namespace waterfurnace_aurora
}  // namespace esphome
