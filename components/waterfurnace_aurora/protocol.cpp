#include "protocol.h"

#include <algorithm>
#include <cstring>

namespace esphome {
namespace waterfurnace_aurora {
namespace protocol {

// --- CRC-16 (standard Modbus, polynomial 0xA001, init 0xFFFF) ---

uint16_t crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0xFFFF;
  if (data == nullptr || len == 0) return crc;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

// --- Helper: append CRC to a frame ---

static void append_crc(std::vector<uint8_t> &frame) {
  uint16_t crc = crc16(frame.data(), frame.size());
  frame.push_back(crc & 0xFF);         // CRC low byte first (Modbus convention)
  frame.push_back((crc >> 8) & 0xFF);
}

// --- Frame building ---

std::vector<uint8_t> build_read_ranges_request(
    uint8_t slave_addr,
    const std::vector<std::pair<uint16_t, uint16_t>> &ranges) {
  if (ranges.empty()) return {};

  std::vector<uint8_t> frame;
  frame.reserve(2 + ranges.size() * 4 + 2);

  frame.push_back(slave_addr);
  frame.push_back(FUNC_READ_RANGES);

  for (const auto &range : ranges) {
    frame.push_back(range.first >> 8);
    frame.push_back(range.first & 0xFF);
    frame.push_back(range.second >> 8);
    frame.push_back(range.second & 0xFF);
  }

  append_crc(frame);
  return frame;
}

std::vector<uint8_t> build_read_registers_request(
    uint8_t slave_addr,
    const uint16_t *addresses, size_t count) {
  if (addresses == nullptr || count == 0 || count > MAX_REGISTERS_PER_REQUEST) return {};

  std::vector<uint8_t> frame;
  frame.reserve(2 + count * 2 + 2);

  frame.push_back(slave_addr);
  frame.push_back(FUNC_READ_REGISTERS);

  for (size_t i = 0; i < count; i++) {
    frame.push_back(addresses[i] >> 8);
    frame.push_back(addresses[i] & 0xFF);
  }

  append_crc(frame);
  return frame;
}

std::vector<uint8_t> build_read_holding_request(
    uint8_t slave_addr,
    uint16_t start_addr, uint16_t count) {
  std::vector<uint8_t> frame;
  frame.reserve(8);

  frame.push_back(slave_addr);
  frame.push_back(FUNC_READ_HOLDING);
  frame.push_back(start_addr >> 8);
  frame.push_back(start_addr & 0xFF);
  frame.push_back(count >> 8);
  frame.push_back(count & 0xFF);

  append_crc(frame);
  return frame;
}

std::vector<uint8_t> build_write_single_request(
    uint8_t slave_addr,
    uint16_t address, uint16_t value) {
  std::vector<uint8_t> frame;
  frame.reserve(8);

  frame.push_back(slave_addr);
  frame.push_back(FUNC_WRITE_SINGLE);
  frame.push_back(address >> 8);
  frame.push_back(address & 0xFF);
  frame.push_back(value >> 8);
  frame.push_back(value & 0xFF);

  append_crc(frame);
  return frame;
}

std::vector<uint8_t> build_write_multi_request(
    uint8_t slave_addr,
    const std::vector<std::pair<uint16_t, uint16_t>> &writes) {
  if (writes.empty()) return {};

  // Func 0x43 frame: [slave][0x43][byte_count][addr_hi][addr_lo][val_hi][val_lo]...[CRC]
  // byte_count = writes.size() * 4 (each write is 2 bytes addr + 2 bytes value)
  std::vector<uint8_t> frame;
  frame.reserve(3 + writes.size() * 4 + 2);

  frame.push_back(slave_addr);
  frame.push_back(FUNC_WRITE_MULTI);
  frame.push_back(static_cast<uint8_t>(writes.size() * 4));

  for (const auto &w : writes) {
    frame.push_back(w.first >> 8);
    frame.push_back(w.first & 0xFF);
    frame.push_back(w.second >> 8);
    frame.push_back(w.second & 0xFF);
  }

  append_crc(frame);
  return frame;
}

// --- Frame validation ---

bool validate_frame_crc(const uint8_t *data, size_t len) {
  if (data == nullptr || len < MIN_FRAME_SIZE) return false;
  uint16_t received_crc = (static_cast<uint16_t>(data[len - 1]) << 8) | data[len - 2];
  uint16_t calc_crc = crc16(data, len - 2);
  return received_crc == calc_crc;
}

// --- Expected frame size ---

size_t expected_frame_size(const uint8_t *data, size_t available) {
  if (available < 2) return 0;  // Need at least addr + func

  uint8_t func = data[1];

  // Error response: [addr][func|0x80][error_code][CRC_lo][CRC_hi] = 5 bytes
  if (func & ERROR_MASK) {
    return 5;
  }

  // Write single echo: [addr][0x06][addr_hi][addr_lo][val_hi][val_lo][CRC_lo][CRC_hi] = 8 bytes
  if (func == FUNC_WRITE_SINGLE) {
    return 8;
  }

  // Read responses and write-multi echo: [addr][func][byte_count][data...][CRC_lo][CRC_hi]
  // Need byte_count (byte 2) to determine total size
  if (func == FUNC_READ_HOLDING || func == FUNC_READ_RANGES ||
      func == FUNC_READ_REGISTERS || func == FUNC_WRITE_MULTI) {
    if (available < 3) return 0;  // Need byte_count
    uint8_t byte_count = data[2];
    return 3 + byte_count + 2;  // header + data + CRC
  }

  // Unknown function code — can't determine size
  return 0;
}

// --- Response parsing ---

ParsedResponse parse_frame(
    const uint8_t *frame, size_t frame_len,
    const uint16_t *expected_addresses, size_t expected_count) {
  ParsedResponse resp;

  if (frame == nullptr || frame_len < MIN_FRAME_SIZE) {
    resp.is_error = true;
    resp.error_code = 0xFF;  // Internal: frame too short
    return resp;
  }

  // Validate CRC
  if (!validate_frame_crc(frame, frame_len)) {
    resp.is_error = true;
    resp.error_code = 0xFE;  // Internal: CRC mismatch
    return resp;
  }

  resp.function_code = frame[1];

  // Error response
  if (resp.function_code & ERROR_MASK) {
    resp.is_error = true;
    resp.error_code = (frame_len >= 3) ? frame[2] : 0;
    return resp;
  }

  // Write single echo — no register data to extract
  if (resp.function_code == FUNC_WRITE_SINGLE) {
    return resp;
  }

  // Read responses: extract register values
  if (resp.function_code == FUNC_READ_HOLDING ||
      resp.function_code == FUNC_READ_RANGES ||
      resp.function_code == FUNC_READ_REGISTERS ||
      resp.function_code == FUNC_WRITE_MULTI) {
    if (frame_len < 5) {
      resp.is_error = true;
      resp.error_code = 0xFF;
      return resp;
    }

    uint8_t byte_count = frame[2];
    size_t num_values = byte_count / 2;
    size_t data_start = 3;

    // Verify we have enough data
    if (frame_len < data_start + byte_count + 2) {
      resp.is_error = true;
      resp.error_code = 0xFD;  // Internal: truncated
      return resp;
    }

    resp.registers.reserve(num_values);

    if (expected_addresses != nullptr && expected_count > 0) {
      // Map values back to expected addresses (for func 0x42, 0x41, 0x03)
      size_t count = std::min(num_values, expected_count);
      for (size_t i = 0; i < count; i++) {
        uint16_t value = (static_cast<uint16_t>(frame[data_start + i * 2]) << 8) |
                         frame[data_start + i * 2 + 1];
        resp.registers.push_back({expected_addresses[i], value});
      }
    } else {
      // No expected addresses — return values with index as address (caller's problem)
      for (size_t i = 0; i < num_values; i++) {
        uint16_t value = (static_cast<uint16_t>(frame[data_start + i * 2]) << 8) |
                         frame[data_start + i * 2 + 1];
        resp.registers.push_back({static_cast<uint16_t>(i), value});
      }
    }
  }

  return resp;
}

// --- Breakpoint-aware helpers ---

std::array<std::vector<uint16_t>, 3> split_by_segment(
    const std::vector<uint16_t> &addresses) {
  std::array<std::vector<uint16_t>, 3> segments;

  for (uint16_t addr : addresses) {
    segments[address_segment(addr)].push_back(addr);
  }

  // Sort each segment
  for (auto &seg : segments) {
    std::sort(seg.begin(), seg.end());
  }

  return segments;
}

std::vector<std::pair<uint16_t, uint16_t>> merge_to_ranges(
    const std::vector<uint16_t> &sorted_addresses, uint16_t max_gap) {
  std::vector<std::pair<uint16_t, uint16_t>> ranges;
  if (sorted_addresses.empty()) return ranges;

  uint16_t range_start = sorted_addresses[0];
  uint16_t range_end = sorted_addresses[0];

  for (size_t i = 1; i < sorted_addresses.size(); i++) {
    if (sorted_addresses[i] - range_end <= max_gap) {
      range_end = sorted_addresses[i];
    } else {
      ranges.emplace_back(range_start, range_end - range_start + 1);
      range_start = sorted_addresses[i];
      range_end = sorted_addresses[i];
    }
  }
  ranges.emplace_back(range_start, range_end - range_start + 1);

  return ranges;
}

}  // namespace protocol
}  // namespace waterfurnace_aurora
}  // namespace esphome
