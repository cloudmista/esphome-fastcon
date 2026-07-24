#include "fastcon_controller.h"
#include "esphome/core/log.h"
#include "esphome/components/light/light_state.h"
#include "protocol.h"
#include "esp_coexist.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string>

namespace esphome {
namespace fastcon {

static const char *const TAG = "fastcon.controller";

void FastconController::setup() {
    ESP_LOGCONFIG(TAG, "Fastcon Controller: High-Frequency Burst Mode Active");
}

// --- High Performance Queue Logic ---

void FastconController::single_control(uint32_t light_id, const std::vector<uint8_t> &light_data) {
    ESP_LOGD(TAG, "[CALL] single_control() entry at %u us for light_id=%u", micros(), (unsigned) light_id);
    std::vector<uint8_t> result_data(12);
    // Protocol header for single light control
    result_data[0] = 2 | (((0x0FFFFFF & (light_data.size() + 1)) << 4));
    result_data[1] = (uint8_t)light_id;
    std::copy(light_data.begin(), light_data.end(), result_data.begin() + 2);

    // Encrypt and wrap into BLE packet
    std::vector<uint8_t> final_packet = this->generate_command(5, light_id, result_data, true);
    this->queueCommand(light_id, final_packet);
}

void FastconController::queueCommand(uint32_t light_id, const std::vector<uint8_t> &data) {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    
    // Clear queue if it gets backed up to ensure the latest command is instant
    if (this->queue_.size() > 10) {
        while(!this->queue_.empty()) this->queue_.pop();
    }

    Command cmd;
    cmd.data = data;
    cmd.retries = 0;
    this->queue_.push(cmd);
}

void FastconController::loop() {
    // NEW investigation angle: WiFi and the BT controller share ONE
    // physical 2.4GHz radio on the ESP32-S3, regardless of which CPU
    // core their tasks run on (core-pinning didn't fix the delay -
    // consistent with this being a radio-arbitration problem, not a
    // CPU-scheduling one). By default ESP-IDF's coexistence scheduler
    // uses ESP_COEX_PREFER_BALANCE, so an incoming WiFi/TCP packet
    // (an HA command) can lose airtime arbitration to an in-flight
    // BLE advertising burst or scan window. Biasing towards WiFi
    // should let API traffic through promptly while fastcon's BLE
    // bursts still get through, just without starving WiFi.
    //
    // Re-applied/re-logged every 30s (not just once on boot) - the device's
    // log ring buffer is only 768 bytes and the log-viewer connection can
    // take 10+ seconds to establish, so a one-shot boot-time log line can
    // easily be evicted before anyone's actually watching. Repeating it
    // makes it impossible to miss, and re-calling the setter is harmless.
    uint32_t now_coex = millis();
    if (now_coex - this->last_coex_set_ms_ >= 30000) {
        this->last_coex_set_ms_ = now_coex;
        esp_err_t err = esp_coex_preference_set(ESP_COEX_PREFER_WIFI);
        ESP_LOGW(TAG, "[COEX] esp_coex_preference_set(ESP_COEX_PREFER_WIFI) -> %d", (int) err);
    }

    // DIAGNOSTIC: command delays on an already-open connection show ZERO
    // TCP packet loss (confirmed via tcpdump), so the delay must be inside
    // the ESP's own processing - most likely some FreeRTOS task starving
    // the CPU right when a command needs to be dispatched. Logs each
    // task's CPU-time delta (microseconds) since the last sample, every
    // 3s, so a monopolizing task shows up directly instead of being
    // washed out by a lifetime-since-boot average.
    uint32_t now_ts = millis();
    if (now_ts - this->last_taskstats_ms_ >= 3000) {
        this->last_taskstats_ms_ = now_ts;
        this->log_task_stats_();
    }

    uint32_t now = millis();

    switch (this->adv_state_) {
        case AdvertiseState::IDLE: {
            std::lock_guard<std::mutex> lock(this->queue_mutex_);
            if (!this->queue_.empty()) {
                ESP_LOGD(TAG, "[STATE] IDLE -> ADVERTISING at %u us, queue_size=%u, retries=0",
                         micros(), (unsigned) this->queue_.size());
                this->adv_state_ = AdvertiseState::ADVERTISING;
                this->state_start_time_ = now;
                this->start_advertising_(this->queue_.front().data);
            }
            break;
        }
        case AdvertiseState::ADVERTISING: {
            if (now - this->state_start_time_ >= this->adv_duration_) {
                ESP_LOGD(TAG, "[STATE] ADVERTISING -> GAP at %u us", micros());
                this->stop_advertising_();
                this->adv_state_ = AdvertiseState::GAP;
                this->state_start_time_ = now;
            }
            break;
        }
        case AdvertiseState::GAP: {
            if (now - this->state_start_time_ >= this->adv_gap_) {
                std::lock_guard<std::mutex> lock(this->queue_mutex_);
                if (!this->queue_.empty()) {
                    auto &cmd = this->queue_.front();
                    cmd.retries++;
                    if (cmd.retries >= Command::MAX_RETRIES) {
                        ESP_LOGD(TAG, "[STATE] GAP -> IDLE at %u us, retries exhausted (%u/%u), popping command",
                                 micros(), cmd.retries, (unsigned) Command::MAX_RETRIES);
                        this->queue_.pop();
                        this->adv_state_ = AdvertiseState::IDLE;
                    } else {
                        ESP_LOGD(TAG, "[STATE] GAP -> ADVERTISING at %u us, retries=%u/%u",
                                 micros(), cmd.retries, (unsigned) Command::MAX_RETRIES);
                        this->adv_state_ = AdvertiseState::ADVERTISING;
                        this->state_start_time_ = now;
                        this->start_advertising_(cmd.data);
                    }
                }
            }
            break;
        }
    }
}

// --- Protocol Helper: Encryption & Formatting ---

std::vector<uint8_t> FastconController::generate_command(uint8_t n, uint32_t light_id, const std::vector<uint8_t> &data, bool forward) {
    static uint8_t sequence = 1;
    std::vector<uint8_t> body(data.size() + 4);
    
    uint8_t i2 = (light_id / 256);
    body[0] = (i2 & 0b1111) | ((n & 0b111) << 4) | (forward ? 0x80 : 0);
    body[1] = sequence++;
    if (sequence == 0) sequence = 1;
    body[2] = this->mesh_key_[3];

    std::copy(data.begin(), data.end(), body.begin() + 4);

    uint8_t checksum = 0;
    for (size_t i = 0; i < body.size(); i++) {
        if (i != 3) checksum += body[i];
    }
    body[3] = checksum;

    // Encrypt with Mesh Key
    for (size_t i = 0; i < 4; i++) body[i] ^= DEFAULT_ENCRYPT_KEY[i & 3];
    for (size_t i = 0; i < data.size(); i++) body[4 + i] ^= this->mesh_key_[i & 3];

    std::vector<uint8_t> addr = {DEFAULT_BLE_FASTCON_ADDRESS.begin(), DEFAULT_BLE_FASTCON_ADDRESS.end()};
    return prepare_payload(addr, body);
}

// --- Data Helpers: Converting ESPHome State to Fastcon Bytes ---

static inline uint8_t to8(float v) { return static_cast<uint8_t>(esphome::clamp(v, 0.0f, 1.0f) * 255.0f); }

std::vector<uint8_t> FastconController::get_light_data(light::LightState *state) {
    auto values = state->current_values;
    if (!values.is_on()) return {0x00};

    float r, g, b, cw, ww;
    state->current_values_as_rgbww(&r, &g, &b, &cw, &ww, false);

    uint8_t brightness = static_cast<uint8_t>(values.get_brightness() * 127.0f);
    return { static_cast<uint8_t>(0x80 | brightness), to8(b), to8(r), to8(g), to8(ww), to8(cw) };
}

std::vector<uint8_t> FastconController::get_white_light_data(light::LightState *state) {
    auto values = state->current_values;
    if (!values.is_on()) return {0x00};

    uint8_t brightness = static_cast<uint8_t>(values.get_brightness() * 127.0f);
    // Force cold/warm white channels to max for "white mode"
    return { static_cast<uint8_t>(0x80 | brightness), 0, 0, 0, 127, 127 };
}

// --- BLE Hardware Interface ---

void FastconController::start_advertising_(const std::vector<uint8_t> &data) {
    this->last_start_call_us_ = micros();
    ESP_LOGD(TAG, "[CALL] start_advertising_() entry at %u us", this->last_start_call_us_);

    esp_ble_adv_params_t adv_params = {
        .adv_int_min = adv_interval_min_,
        .adv_int_max = adv_interval_max_,
        .adv_type = ADV_TYPE_NONCONN_IND,
        .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .peer_addr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
        .peer_addr_type = BLE_ADDR_TYPE_PUBLIC,
        .channel_map = ADV_CHNL_ALL,
        .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    };

    uint8_t adv_data_raw[31] = {0};
    uint8_t len = 0;
    adv_data_raw[len++] = 2;
    adv_data_raw[len++] = ESP_BLE_AD_TYPE_FLAG;
    adv_data_raw[len++] = ESP_BLE_ADV_FLAG_BREDR_NOT_SPT | ESP_BLE_ADV_FLAG_GEN_DISC;
    adv_data_raw[len++] = data.size() + 2;
    adv_data_raw[len++] = ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE;
    adv_data_raw[len++] = 0xF0; 
    adv_data_raw[len++] = 0xFF;
    memcpy(&adv_data_raw[len], data.data(), data.size());
    len += data.size();

    uint32_t before_config = micros();
    esp_ble_gap_config_adv_data_raw(adv_data_raw, len);
    uint32_t before_start = micros();
    esp_ble_gap_start_advertising(&adv_params);
    uint32_t after_start = micros();
    ESP_LOGD(TAG, "[CALL] config_adv_data_raw() took %u us, start_advertising() took %u us (call itself, not the *_COMPLETE_EVT)",
             before_start - before_config, after_start - before_start);
}

void FastconController::stop_advertising_() {
    this->last_stop_call_us_ = micros();
    ESP_LOGD(TAG, "[CALL] stop_advertising_() entry at %u us", this->last_stop_call_us_);
    esp_ble_gap_stop_advertising();
    ESP_LOGD(TAG, "[CALL] stop_advertising() call itself took %u us (not the *_COMPLETE_EVT)",
             micros() - this->last_stop_call_us_);
}

// --- DIAGNOSTIC: FreeRTOS per-task CPU usage delta ---

void FastconController::log_task_stats_() {
    static constexpr UBaseType_t MAX_TASKS = 32;
    static TaskStatus_t status[MAX_TASKS];
    static TaskHandle_t prev_handle[MAX_TASKS] = {};
    static uint32_t prev_runtime[MAX_TASKS] = {};
    static UBaseType_t prev_count = 0;

    uint32_t total_runtime = 0;
    UBaseType_t count = uxTaskGetSystemState(status, MAX_TASKS, &total_runtime);

    std::string out = "[TASKSTATS] CPU us consumed in last ~3s, per task:";
    char entry[48];
    for (UBaseType_t i = 0; i < count; i++) {
        uint32_t cur = status[i].ulRunTimeCounter;
        uint32_t delta = 0;
        for (UBaseType_t j = 0; j < prev_count; j++) {
            if (prev_handle[j] == status[i].xHandle) {
                delta = cur - prev_runtime[j];
                break;
            }
        }
        if (delta > 0) {
            snprintf(entry, sizeof(entry), " %s=%u", status[i].pcTaskName, (unsigned) delta);
            out += entry;
        }
    }
    ESP_LOGW(TAG, "%s", out.c_str());

    for (UBaseType_t i = 0; i < count && i < MAX_TASKS; i++) {
        prev_handle[i] = status[i].xHandle;
        prev_runtime[i] = status[i].ulRunTimeCounter;
    }
    prev_count = count;
}

} // namespace fastcon
} // namespace esphome
