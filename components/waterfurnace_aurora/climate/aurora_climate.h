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
  void set_zone(uint8_t zone) { this->zone_ = zone; }

  // Climate traits
  climate::ClimateTraits traits() override;

  // Climate control
  void control(const climate::ClimateCall &call) override;

 protected:
  void register_listeners_();
  void update_state_();
  void publish_state_if_changed_();

  /// Zone 1 auto-detects thermostat vs IZ2 at runtime.
  /// Zones 2-6 are always IZ2.
  bool is_iz2_mode_() const {
    return this->zone_ > 1 || (this->zone_ == 1 && this->parent_->has_iz2());
  }

  WaterFurnaceAurora *parent_{nullptr};
  uint8_t zone_{1};

  // Dedup tracking — prevents redundant publish_state() calls when nothing changed.
  float last_current_temp_{NAN};
  float last_current_humidity_{NAN};
  float last_target_low_{NAN};
  float last_target_high_{NAN};
  float last_target_humidity_{NAN};
  climate::ClimateMode last_mode_{climate::CLIMATE_MODE_OFF};
  climate::ClimateAction last_action_{climate::CLIMATE_ACTION_OFF};
  optional<climate::ClimateFanMode> last_fan_mode_{};
  const char *last_custom_fan_mode_{nullptr};
  climate::ClimatePreset last_preset_{climate::CLIMATE_PRESET_NONE};
  bool has_published_{false};
};

}  // namespace waterfurnace_aurora
}  // namespace esphome
