#pragma once

#include <queue>
#include <mutex>
#include <vector>
#include "esphome/core/component.h"
#include "esphome/components/esp32_ble_server/ble_server.h"
#include "esphome/components/esp32_ble/ble.h"

namespace esphome {
namespace light { class LightState; }

namespace fastcon {

// DIAGNOSTIC BUILD: instrumented to localize a consistent 6-7s delay between
// a light command being issued and it actually taking effect. Registers as a
// esp32_ble::GAPEventHandler (via the shared global_ble dispatcher, NOT a
// direct esp_ble_gap_register_callback() call, since that would silently
// replace esp32_ble_server's own registration) to log the real-world gap
// between issuing start_advertising_()/stop_advertising_() and the
// corresponding *_COMPLETE_EVT actually arriving - neither of which this
// controller previously waited for or even observed.
class FastconController : public Component, public esp32_ble::GAPEventHandler {
 public:
  void setup() override;
  void loop() override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

  // MUST run after esp32_ble::ESP32BLE::setup() (priority BLUETOOTH, 350.0f),
  // since that's what assigns the global_ble pointer this component's setup()
  // dereferences via register_gap_event_handler(). Component defaults to
  // DATA (600.0f) - i.e. runs BEFORE BLUETOOTH - which crashed on every boot
  // (null global_ble) until this override was added.
  float get_setup_priority() const override { return esphome::setup_priority::AFTER_BLUETOOTH; }

  // YAML Setters (Fixes the main.cpp errors)
  void set_adv_interval(uint16_t val) { adv_interval_min_ = adv_interval_max_ = val; }
  void set_adv_interval_min(uint16_t val) { adv_interval_min_ = val; }
  void set_adv_interval_max(uint16_t val) { adv_interval_max_ = val; }
  void set_adv_duration(uint16_t val) { adv_duration_ = val; }
  void set_adv_gap(uint16_t val) { adv_gap_ = val; }
  void set_max_queue_size(size_t size) { max_queue_size_ = size; }

  // Protocol & Queueing
  void single_control(uint32_t light_id, const std::vector<uint8_t> &light_data);
  void queueCommand(uint32_t light_id, const std::vector<uint8_t> &data);
  std::vector<uint8_t> get_light_data(light::LightState *state);
  std::vector<uint8_t> get_white_light_data(light::LightState *state);

  void set_mesh_key(std::array<uint8_t, 4> key) { mesh_key_ = key; }

 protected:
  struct Command {
    std::vector<uint8_t> data;
    uint8_t retries{0};
    // Was 20 - measured at ~0.9-1.75s of real wall-clock time to exhaust per
    // command (each retry cycle runs slower than the coded 45ms due to GAP
    // event dispatch overhead). With no ACK in this protocol, retries past
    // the first few buy little extra reliability while linearly multiplying
    // per-command cost - and that cost multiplies again by queue depth
    // (e.g. all 8 lights restoring state at once on reconnect took ~7.2s
    // total to drain at 20 retries each). Dropped to 3.
    static constexpr uint8_t MAX_RETRIES = 3;
  };

  std::queue<Command> queue_;
  mutable std::mutex queue_mutex_;
  size_t max_queue_size_{100};
  
  enum class AdvertiseState { IDLE, ADVERTISING, GAP };
  AdvertiseState adv_state_{AdvertiseState::IDLE};
  uint32_t state_start_time_{0};

  void start_advertising_(const std::vector<uint8_t> &data);
  void stop_advertising_();
  std::vector<uint8_t> generate_command(uint8_t n, uint32_t light_id, const std::vector<uint8_t> &data, bool forward);

  // DIAGNOSTIC: timestamp (micros()) of the most recent esp_ble_gap_start_advertising()
  // / esp_ble_gap_stop_advertising() call, so gap_event_handler() can log how long the
  // stack actually took to fire the matching *_COMPLETE_EVT.
  uint32_t last_start_call_us_{0};
  uint32_t last_stop_call_us_{0};

  std::array<uint8_t, 4> mesh_key_{};
  uint16_t adv_interval_min_{0x20}; 
  uint16_t adv_interval_max_{0x20};
  uint16_t adv_duration_{40}; 
  uint16_t adv_gap_{5};
};

} // namespace fastcon
} // namespace esphome
