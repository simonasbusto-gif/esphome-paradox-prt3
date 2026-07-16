#pragma once
#include <cstring>
#include <string>
#include <cstdio>
#include <cctype>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace paradox_prt3 {

class ParadoxPRT3 : public Component, public uart::UARTDevice {
 public:
  explicit ParadoxPRT3(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

  void set_last_message(text_sensor::TextSensor *s) { last_message_ = s; }
  void set_alarm_state(text_sensor::TextSensor *s) { alarm_state_ = s; }
  void set_last_error(text_sensor::TextSensor *s) { last_error_ = s; }

  void add_zone(binary_sensor::BinarySensor *s) { zones_.push_back(s); }

  void set_ready(binary_sensor::BinarySensor *s) { ready_ = s; }
  void set_trouble(binary_sensor::BinarySensor *s) { trouble_ = s; }
  void set_memory(binary_sensor::BinarySensor *s) { memory_ = s; }
  void set_strobe(binary_sensor::BinarySensor *s) { strobe_ = s; }

  void arm_area(int area, const std::string &mode, const std::string &code) {
    if (last_error_ != nullptr) last_error_->publish_state("");

    pending_cmd_ = PendingCmd::ARM;
    pending_since_ms_ = millis();

    char cmd[32];
    int a = clamp_area_(area);
    char m = normalize_mode_(mode);

    std::snprintf(cmd, sizeof(cmd), "AA%03d%c%s\r", a, m, code.c_str());
    send_str_(cmd);

    request_area_status(a);
    last_poll_ = millis();
  }

  void disarm_area(int area, const std::string &code) {
    if (last_error_ != nullptr) last_error_->publish_state("");

    pending_cmd_ = PendingCmd::DISARM;
    pending_since_ms_ = millis();

    char cmd[32];
    int a = clamp_area_(area);

    std::snprintf(cmd, sizeof(cmd), "AD%03d%s\r", a, code.c_str());
    send_str_(cmd);

    request_area_status(a);
    last_poll_ = millis();
  }

  void request_area_status(int area) {
    char cmd[16];
    int a = clamp_area_(area);

    std::snprintf(cmd, sizeof(cmd), "RA%03d\r", a);
    send_str_(cmd);

    ESP_LOGI("paradox_prt3", "Sent RA%03d (status request)", a);
  }

  void dump_config() override {
    ESP_LOGCONFIG("paradox_prt3", "ParadoxPRT3:");
    ESP_LOGCONFIG("paradox_prt3", "  version: 40-zones-1s-poll-long-arming-v0.4.0");
    ESP_LOGCONFIG("paradox_prt3", "  last_message set: %s", last_message_ ? "yes" : "no");
    ESP_LOGCONFIG("paradox_prt3", "  alarm_state set: %s", alarm_state_ ? "yes" : "no");
    ESP_LOGCONFIG("paradox_prt3", "  last_error set: %s", last_error_ ? "yes" : "no");
    ESP_LOGCONFIG("paradox_prt3", "  zones configured: %u", static_cast<unsigned>(zones_.size()));
    ESP_LOGCONFIG("paradox_prt3", "  ready set: %s", ready_ ? "yes" : "no");
    ESP_LOGCONFIG("paradox_prt3", "  trouble set: %s", trouble_ ? "yes" : "no");
    ESP_LOGCONFIG("paradox_prt3", "  memory set: %s", memory_ ? "yes" : "no");
    ESP_LOGCONFIG("paradox_prt3", "  strobe set: %s", strobe_ ? "yes" : "no");
  }

  void setup() override {
    // Do NOT force "pending" on ESP boot.
    // Wait for a real RA001 status instead.
    if (last_message_ != nullptr) last_message_->publish_state("");
    if (last_error_ != nullptr) last_error_->publish_state("");

    for (auto *zone : zones_) {
      if (zone != nullptr) zone->publish_state(false);
    }

    if (ready_ != nullptr) ready_->publish_state(true);
    if (trouble_ != nullptr) trouble_->publish_state(false);
    if (memory_ != nullptr) memory_->publish_state(false);
    if (strobe_ != nullptr) strobe_->publish_state(false);

    boot_ms_ = millis();
    got_area_status_ = false;
    no_status_error_sent_ = false;

    pending_cmd_ = PendingCmd::NONE;
    pending_since_ms_ = 0;

    request_area_status(1);
    last_poll_ = millis();
  }

  void loop() override {
    // Poll every 1 second all the time.
    // This makes arm/disarm/triggered status update much faster in HA.
    if (millis() - last_poll_ > STATUS_POLL_MS) {
      last_poll_ = millis();
      request_area_status(1);
    }

    // If PRT3 does not answer status requests after boot, expose this as an error,
    // but do not force alarm_state to pending.
    if (!got_area_status_ && !no_status_error_sent_ && millis() - boot_ms_ > NO_STATUS_ERROR_MS) {
      no_status_error_sent_ = true;

      if (last_error_ != nullptr) {
        last_error_->publish_state("no_status_response");
      }

      ESP_LOGW("paradox_prt3", "No RA001 status response received after ESP boot");
    }

    const int max_line_length = 160;
    static char buffer[max_line_length];

    uint8_t c;
    while (this->available()) {
      if (this->read_byte(&c)) {
        int line_len = readline_(static_cast<int>(c), buffer, max_line_length);

        if (line_len > 0) {
          std::string safe = make_safe_text_(buffer, line_len);

          ESP_LOGD("paradox_prt3", "RX: %s", safe.c_str());

          if (last_message_ != nullptr) {
            last_message_->publish_state(safe);
          }

          if (safe.rfind("HEX:", 0) != 0) {
            handle_line_(safe.c_str());
          }
        }
      }
    }
  }

 protected:
  text_sensor::TextSensor *last_message_{nullptr};
  text_sensor::TextSensor *alarm_state_{nullptr};
  text_sensor::TextSensor *last_error_{nullptr};

  std::vector<binary_sensor::BinarySensor *> zones_;

  binary_sensor::BinarySensor *ready_{nullptr};
  binary_sensor::BinarySensor *trouble_{nullptr};
  binary_sensor::BinarySensor *memory_{nullptr};
  binary_sensor::BinarySensor *strobe_{nullptr};

  uint32_t boot_ms_{0};
  bool got_area_status_{false};
  bool no_status_error_sent_{false};
  uint32_t last_poll_{0};

  enum class PendingCmd { NONE, ARM, DISARM };

  PendingCmd pending_cmd_{PendingCmd::NONE};
  uint32_t pending_since_ms_{0};

  // 1 second polling all the time
  static constexpr uint32_t STATUS_POLL_MS = 1000;

  // Keep HA in "arming" while Paradox is in exit delay.
  // Your panel reports RA001D... during exit delay, so this must be longer than exit delay.
  static constexpr uint32_t TRANSITION_GUARD_MS = 120000;

  static constexpr uint32_t NO_STATUS_ERROR_MS = 60000;

  int readline_(int readch, char *buffer, int len) {
    static int pos = 0;

    if (readch < 0) return -1;

    switch (readch) {
      case '\n':
        break;

      case '\r': {
        int rpos = pos;
        pos = 0;
        return rpos;
      }

      default:
        if (pos < len - 1) {
          buffer[pos++] = static_cast<char>(readch);
          buffer[pos] = 0;
        } else {
          pos = 0;
        }
    }

    return -1;
  }

  void send_str_(const char *s) {
    this->write_array(reinterpret_cast<const uint8_t *>(s), std::strlen(s));
  }

  std::string make_safe_text_(const char *buf, int len) {
    bool printable = true;

    for (int i = 0; i < len; i++) {
      unsigned char b = static_cast<unsigned char>(buf[i]);

      if (!std::isprint(b)) {
        printable = false;
        break;
      }
    }

    if (printable) {
      return std::string(buf, buf + len);
    }

    static const char *hex = "0123456789ABCDEF";
    std::string out = "HEX:";
    out.reserve(4 + len * 3);

    for (int i = 0; i < len; i++) {
      unsigned char b = static_cast<unsigned char>(buf[i]);

      out.push_back(' ');
      out.push_back(hex[(b >> 4) & 0xF]);
      out.push_back(hex[b & 0xF]);
    }

    return out;
  }

  int clamp_area_(int area) {
    if (area < 1) return 1;
    if (area > 8) return 8;
    return area;
  }

  char normalize_mode_(const std::string &mode) {
    if (mode.empty()) return 'A';

    char m = mode[0];

    if (m >= 'a' && m <= 'z') {
      m = static_cast<char>(m - 32);
    }

    if (m == 'A' || m == 'F' || m == 'S' || m == 'I') {
      return m;
    }

    return 'A';
  }

  void set_state_(const char *s) {
    if (alarm_state_ != nullptr) {
      alarm_state_->publish_state(s);
    }
  }

  bool in_transition_guard_() const {
    if (pending_cmd_ == PendingCmd::NONE) return false;
    return (millis() - pending_since_ms_) <= TRANSITION_GUARD_MS;
  }

  int parse_zone_number_(const char *line) {
    // Expected:
    // G001N001A001 = zone 1 open
    // G000N001A001 = zone 1 closed
    if (std::strlen(line) < 12) return -1;
    if (line[0] != 'G') return -1;
    if (line[4] != 'N') return -1;

    if (!std::isdigit(static_cast<unsigned char>(line[5])) ||
        !std::isdigit(static_cast<unsigned char>(line[6])) ||
        !std::isdigit(static_cast<unsigned char>(line[7]))) {
      return -1;
    }

    int zone = (line[5] - '0') * 100 + (line[6] - '0') * 10 + (line[7] - '0');
    return zone;
  }

  bool is_zone_open_event_(const char *line) {
    return strncmp(line, "G001N", 5) == 0;
  }

  bool is_zone_closed_event_(const char *line) {
    return strncmp(line, "G000N", 5) == 0;
  }

  void update_zone_from_event_(const char *line) {
    bool is_open = is_zone_open_event_(line);
    bool is_closed = is_zone_closed_event_(line);

    if (!is_open && !is_closed) return;

    int zone_number = parse_zone_number_(line);
    if (zone_number < 1) return;

    int index = zone_number - 1;

    if (index < 0 || index >= static_cast<int>(zones_.size())) {
      ESP_LOGW("paradox_prt3", "Received zone %d event, but only %u zones configured",
               zone_number, static_cast<unsigned>(zones_.size()));
      return;
    }

    auto *zone = zones_[index];
    if (zone == nullptr) return;

    zone->publish_state(is_open);
  }

  void handle_line_(const char *line) {
    // ---- Command ACK ----
    if (strncmp(line, "AA001&ok", 8) == 0) {
      pending_cmd_ = PendingCmd::ARM;
      pending_since_ms_ = millis();
      set_state_("arming");
      return;
    }

    if (strncmp(line, "AD001&ok", 8) == 0) {
      pending_cmd_ = PendingCmd::DISARM;
      pending_since_ms_ = millis();
      set_state_("disarming");
      return;
    }

    // ---- Command fail ----
    // Only real command failures start with AA or AD.
    // Ignore broken fragments like "0&fail" so they don't create false command_failed.
    if (strstr(line, "&fail") != nullptr) {
      if (strncmp(line, "AA", 2) == 0) {
        if (last_error_ != nullptr) last_error_->publish_state("arm_failed");
        pending_cmd_ = PendingCmd::NONE;
        pending_since_ms_ = 0;
        return;
      }

      if (strncmp(line, "AD", 2) == 0) {
        if (last_error_ != nullptr) last_error_->publish_state("disarm_failed");
        pending_cmd_ = PendingCmd::NONE;
        pending_since_ms_ = 0;
        return;
      }

      ESP_LOGW("paradox_prt3", "Ignoring non-command fail fragment: %s", line);
      return;
    }

    // ---- Universal zone open/closed events ----
    update_zone_from_event_(line);

    // ---- Area 1 status: RA001 + status bytes ----
    if (strncmp(line, "RA001", 5) == 0) {
      got_area_status_ = true;
      no_status_error_sent_ = false;

      if (last_error_ != nullptr && last_error_->state == "no_status_response") {
        last_error_->publish_state("");
      }

      if (std::strlen(line) < 12) return;

      const char mode = line[5];   // Byte 6: D/A/F/S/I
      const char mem  = line[6];   // Byte 7: M/O
      const char trb  = line[7];   // Byte 8: T/O
      const char nrd  = line[8];   // Byte 9: N/O
      const char inal = line[10];  // Byte 11: A/O
      const char strb = line[11];  // Byte 12: S/O

      if (memory_ != nullptr) memory_->publish_state(mem == 'M');
      if (trouble_ != nullptr) trouble_->publish_state(trb == 'T');
      if (ready_ != nullptr) ready_->publish_state(nrd != 'N');
      if (strobe_ != nullptr) strobe_->publish_state(strb == 'S');

      // In alarm always wins
      if (inal == 'A') {
        set_state_("triggered");
        return;
      }

      // Keep "arming" from being overwritten by RA001D... during Paradox exit delay.
      // Paradox can still report D while BabyWare shows exit delay.
      if (in_transition_guard_() && alarm_state_ != nullptr) {
        const std::string cur = alarm_state_->state;

        if (pending_cmd_ == PendingCmd::ARM && cur == "arming") {
          if (mode == 'D') return;
        }

        if (pending_cmd_ == PendingCmd::DISARM && cur == "disarming") {
          if (mode != 'D') return;
        }
      }

      switch (mode) {
        case 'D':
          set_state_("disarm");
          pending_cmd_ = PendingCmd::NONE;
          pending_since_ms_ = 0;
          break;

        case 'A':
          set_state_("arm");
          pending_cmd_ = PendingCmd::NONE;
          pending_since_ms_ = 0;
          break;

        case 'F':
          set_state_("force");
          pending_cmd_ = PendingCmd::NONE;
          pending_since_ms_ = 0;
          break;

        case 'S':
          set_state_("stay");
          pending_cmd_ = PendingCmd::NONE;
          pending_since_ms_ = 0;
          break;

        case 'I':
          set_state_("instant");
          pending_cmd_ = PendingCmd::NONE;
          pending_since_ms_ = 0;
          break;

        default:
          set_state_("unknown");
          break;
      }

      return;
    }
  }
};

}  // namespace paradox_prt3
}  // namespace esphome
