// Minimal ESPHome UART mock for unit testing
#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include <queue>

namespace esphome {
namespace uart {

class UARTDevice {
 public:
  virtual ~UARTDevice() = default;

  // --- Mock control (used by tests) ---
  // Feed bytes that will be returned by read()/available()
  void mock_receive(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) rx_queue_.push(data[i]);
  }
  void mock_receive(const std::vector<uint8_t> &data) {
    for (auto b : data) rx_queue_.push(b);
  }
  // Get bytes that were sent via write_array()
  std::vector<uint8_t> mock_get_transmitted() {
    auto result = tx_buffer_;
    tx_buffer_.clear();
    return result;
  }

  // --- ESPHome UARTDevice API stubs ---
  bool available() { return !rx_queue_.empty(); }
  
  uint8_t read() {
    if (rx_queue_.empty()) return 0;
    uint8_t b = rx_queue_.front();
    rx_queue_.pop();
    return b;
  }
  
  void write_array(const uint8_t *data, size_t len) {
    tx_buffer_.insert(tx_buffer_.end(), data, data + len);
  }
  
  void flush() {}

 protected:
  std::queue<uint8_t> rx_queue_;
  std::vector<uint8_t> tx_buffer_;
};

}  // namespace uart
}  // namespace esphome
