#include "esphome/core/log.h"
#include "esphome/components/light/light_state.h"
#include "fastcon_controller.h"
#include "fastcon_light.h"
#include <cmath>
#include <string>

#ifndef FASTCON_VERSION
#define FASTCON_VERSION "0.3.3-dev"
#endif

namespace esphome {
namespace fastcon {

static const char *const TAG = "fastcon.light";

light::LightTraits FastconLight::get_traits() {
  light::LightTraits t;

  // Define supported color modes based on hardware capabilities
  if (!this->supports_rgb_) {
    // CCT-only hardware (white + warm-white LEDs, no RGB channel at all) -
    // was previously always getting an RGB-inclusive mode regardless of
    // supports_rgb, which told Home Assistant these lights could do full
    // color when the physical bulbs can't. Declare only what's real.
    t.set_supported_color_modes({light::ColorMode::COLD_WARM_WHITE});
  } else if (this->color_interlock_) {
    if (this->supports_cwww_) {
      t.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::COLD_WARM_WHITE});
    } else {
      t.set_supported_color_modes({light::ColorMode::RGB, light::ColorMode::WHITE});
    }
  } else {
    if (this->supports_cwww_) {
      t.set_supported_color_modes({light::ColorMode::RGB_COLD_WARM_WHITE});
    } else {
      t.set_supported_color_modes({light::ColorMode::RGB_WHITE});
    }
  }

  if (this->supports_cwww_) {
    t.set_min_mireds(153.0f); // 6500K
    t.set_max_mireds(500.0f); // 2000K
  }

  return t;
}

void FastconLight::write_state(light::LightState *state) {
  if (this->controller_ == nullptr) {
    ESP_LOGW(TAG, "No controller bound; dropping command for light %u", (unsigned)this->light_id_);
    return;
  }

  std::vector<uint8_t> light_bytes;
  auto &values = state->current_values;

  // 1. Determine if this is a white-only or color command
  // CCT-only hardware never has a meaningful RGB value to check - always
  // take the white/CW-WW path. Otherwise, treat pure white or equal RGB
  // values as a "white" command to trigger the dedicated white LEDs.
  bool is_white_only = !this->supports_rgb_;
  if (this->supports_rgb_) {
    float r = values.get_red();
    float g = values.get_green();
    float b = values.get_blue();
    if (values.get_color_mode() == light::ColorMode::WHITE ||
        (fabs(r - g) < 0.001f && fabs(g - b) < 0.001f)) {
      is_white_only = true;
    }
  }

  // 2. Fetch the appropriate raw data payload from the controller
  if (is_white_only) {
    ESP_LOGV(TAG, "Processing white-only state for light %u", (unsigned)this->light_id_);
    light_bytes = this->controller_->get_white_light_data(state);
  } else {
    ESP_LOGV(TAG, "Processing RGB state for light %u", (unsigned)this->light_id_);
    light_bytes = this->controller_->get_light_data(state);
  }

  // DIAGNOSTIC: dump the actual pre-encryption payload bytes and the raw
  // state ESPHome thinks it's applying - "everything at the logical/retry
  // level looks identical between working and non-working cases" isn't
  // enough to rule out a bug in what's actually computed. If a bug only
  // shows up for certain callers (e.g. automations vs. manual UI calls),
  // it has to be visible here.
  {
    std::string hex;
    char buf[4];
    for (uint8_t b : light_bytes) {
      snprintf(buf, sizeof(buf), "%02X ", b);
      hex += buf;
    }
    ESP_LOGD(TAG, "  raw payload: [ %s] is_on=%d brightness=%.3f", hex.c_str(),
             (int) values.is_on(), values.get_brightness());
  }

  // 3. Hand off the data to the controller's high-speed burst queue
  // This method wraps the bytes, encrypts them, and adds them to the 20x retransmit loop
  this->controller_->single_control(this->light_id_, light_bytes);

  ESP_LOGD(TAG, "Queued burst command: light_id=%u, mode=%s",
           (unsigned)this->light_id_, is_white_only ? "WHITE" : "RGB");
}

}  // namespace fastcon
}  // namespace esphome
