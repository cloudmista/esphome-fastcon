
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/light/light_output.h"

namespace esphome {
namespace fastcon {

class FastconController; // fwd decl

class FastconLight : public Component, public light::LightOutput {
 public:
  FastconLight() = default;
  explicit FastconLight(int light_id) { this->light_id_ = static_cast<uint8_t>(light_id); }

  void set_controller(FastconController *c) { controller_ = c; }
  void set_light_id(uint8_t id) { light_id_ = id; }
  void set_supports_cwww(bool v) { supports_cwww_ = v; }
  void set_color_interlock(bool v) { color_interlock_ = v; }
  void set_supports_rgb(bool v) { supports_rgb_ = v; }

  // LightOutput interface
  light::LightTraits get_traits() override;
  void write_state(light::LightState *state) override;

 protected:
  FastconController *controller_{nullptr};
  uint8_t light_id_{0};
  bool supports_cwww_{false};
  bool color_interlock_{false};
  // Defaults true for backwards compatibility with existing full-RGBCWWW
  // fixtures using this fork. Set false for CCT-only hardware (white +
  // warm-white LEDs, no RGB channel at all) - forces the light to always
  // use the white/CW-WW data path, never attempt an RGB command.
  bool supports_rgb_{true};
};

}  // namespace fastcon
}  // namespace esphome
