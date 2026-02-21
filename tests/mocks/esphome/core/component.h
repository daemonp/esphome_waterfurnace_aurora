// Minimal ESPHome mock for unit testing
#pragma once
#include <cstdint>
#include <string>

// Stub LOG_STR macro
#define LOG_STR(s) s

namespace esphome {

namespace setup_priority {
  static constexpr float BUS = 1000.0f;
  static constexpr float IO = 900.0f;
  static constexpr float HARDWARE = 800.0f;
  static constexpr float DATA = 600.0f;
  static constexpr float PROCESSOR = 400.0f;
  static constexpr float BLUETOOTH = 350.0f;
  static constexpr float WIFI = 250.0f;
  static constexpr float AFTER_CONNECTION = 100.0f;
  static constexpr float LATE = -100.0f;
}

class Component {
 public:
  virtual ~Component() = default;
  virtual void setup() {}
  virtual void loop() {}
  virtual void dump_config() {}
  virtual void on_shutdown() {}
  virtual float get_setup_priority() const { return 0.0f; }
  
  void status_set_error(const char * /*msg*/ = nullptr) { status_error_ = true; }
  void status_set_warning(const char * /*msg*/ = nullptr) { status_warning_ = true; }
  void status_clear_error() { status_error_ = false; }
  void status_clear_warning() { status_warning_ = false; }
  
  bool status_has_error() const { return status_error_; }
  bool status_has_warning() const { return status_warning_; }
  
 protected:
  bool status_error_{false};
  bool status_warning_{false};
};

class PollingComponent : public Component {
 public:
  virtual void update() {}
  uint32_t get_update_interval() const { return update_interval_; }
  void set_update_interval(uint32_t interval) { update_interval_ = interval; }
 protected:
  uint32_t update_interval_{5000};
};

// Stub millis() — test code can override via set_millis()
uint32_t &mock_millis_ref();
inline uint32_t millis() { return mock_millis_ref(); }
inline void set_millis(uint32_t ms) { mock_millis_ref() = ms; }

// Stubs for delay/yield
inline void delay(uint32_t /*ms*/) {}
inline void delayMicroseconds(uint32_t /*us*/) {}
inline void yield() {}

}  // namespace esphome
