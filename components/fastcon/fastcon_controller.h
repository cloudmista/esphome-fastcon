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

// DIAGNOSTIC BUILD: timestamp logging (see start_advertising_/stop_advertising_/
// single_control) added to localize a consistent 6-7s delay between a light
// command being issued and it actually taking effect. An earlier version of
// this also hooked esp32_ble::GAPEventHandler to time the *_COMPLETE_EVT
// callbacks directly, but that API doesn't exist in this form on newer
// ESPHome/esp32_ble releases (gap_event_handler is now static/protected on
// ESP32BLE itself) - removed rather than chase a moving target, since the
// call-entry timestamps already answer the question we needed.
class FastconController : public Component {
 public:
  void setup() override;
  void loop() override;

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
    // Was 20, dropped to 3 for latency reasons, briefly raised to 8 after
    // sniffing the real BRmesh app's BLE traffic (which seemed to use more
    // repeats per command) - reverted back to 3. The real app doesn't also
    // sit behind an HA automation that independently triple-sends every
    // command ("repeat: count: 3" in the stairs automation). Stacking our
    // own 8 retries on top of that automation-level redundancy meant each
    // command took ~360ms to drain instead of ~135ms; with 8 lights x 3
    // automation-level repeats arriving in a burst, the queue backed up
    // past its own overflow-clear threshold (see queueCommand() - wipes
    // itself entirely past 10 pending commands), which is exactly what
    // caused lights to start getting skipped. The automation's own repeat
    // layer already provides the extra reliability margin the real app
    // gets from more BLE-level retries - this firmware doesn't need both.
    // Was 20 - measured at ~0.9-1.75s of real wall-clock time to exhaust per
    // command (each retry cycle runs slower than the coded 45ms due to GAP
    // event dispatch overhead). With no ACK in this protocol, retries past
    // the first few buy little extra reliability while linearly multiplying
    // per-command cost - and that cost multiplies again by queue depth
    // (e.g. all 8 lights restoring state at once on reconnect took ~7.2s
    // total to drain at 20 retries each).
    static constexpr uint8_t MAX_RETRIES = 3;
  };

  std::queue<Command> queue_;
  mutable std::mutex queue_mutex_;
  size_t max_queue_size_{100};
  
  enum class AdvertiseState { IDLE, ADVERTISING, GAP };
  AdvertiseState adv_state_{AdvertiseState::IDLE};
  uint32_t state_start_time_{0};

  // Re-applied every 30s from loop() (see .cpp) rather than once on boot -
  // initialized so the very first check in loop() fires immediately
  // (unsigned wraparound: 0 - (uint32_t)(-30000) == 30000).
  uint32_t last_coex_set_ms_{(uint32_t) (-30000)};

  // DIAGNOSTIC: periodic FreeRTOS per-task CPU usage logging (see
  // log_task_stats_() in .cpp) - investigating command delays that show
  // zero network packet loss, to see if some task is monopolizing the CPU
  // when a command is slow to be processed. Same immediate-first-fire
  // wraparound trick as last_coex_set_ms_.
  uint32_t last_taskstats_ms_{(uint32_t) (-3000)};
  void log_task_stats_();

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
