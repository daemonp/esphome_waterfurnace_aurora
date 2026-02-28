#pragma once

// WaterFurnace Aurora Modbus RTU Protocol Module
//
// Standalone, unit-testable module for WaterFurnace custom Modbus framing.
// NO ESPHome dependencies — pure C++ with standard library only.
//
// WaterFurnace uses standard Modbus CRC-16 but custom function codes:
//   0x41 ('A') — Read multiple contiguous register ranges
//   0x42 ('B') — Read specific discontiguous registers
//   0x03       — Standard read holding registers
//   0x06       — Standard write single register
//   0x43       — Write multiple discontiguous registers (func 67, batch write)
//
// Improvements over rw-wf's protocol.cpp (143 lines):
//   - Request validation (reject >100 registers, empty requests)
//   - ParsedResponse struct with RegisterValue address-value pairs
//     (rw-wf returns raw uint16_t vector, caller must correlate)
//   - expected_frame_size() for non-blocking frame completeness checking
//     (rw-wf duplicates this in the hub)
//   - Breakpoint-aware helpers (address_segment, split_by_segment, merge_to_ranges)
//     (rw-wf puts these in the hub; they're protocol knowledge)

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <vector>

namespace esphome {
namespace waterfurnace_aurora {
namespace protocol {

// --- Protocol constants ---
static constexpr uint8_t FUNC_READ_RANGES = 0x41;     // 'A' — read contiguous ranges
static constexpr uint8_t FUNC_READ_REGISTERS = 0x42;  // 'B' — read specific addresses
static constexpr uint8_t FUNC_READ_HOLDING = 0x03;    // Standard read holding registers
static constexpr uint8_t FUNC_WRITE_SINGLE = 0x06;    // Standard write single register
static constexpr uint8_t FUNC_WRITE_MULTI = 0x43;     // Func 67 — batch write multiple
static constexpr uint8_t ERROR_MASK = 0x80;
static constexpr size_t MAX_REGISTERS_PER_REQUEST = 100;
static constexpr size_t MIN_FRAME_SIZE = 4;   // addr + func + 2 CRC
static constexpr size_t MAX_FRAME_SIZE = 512;

// WaterFurnace protocol breakpoints — queries cannot span these boundaries.
// The ABC board treats register space as three segments with distinct address
// ranges. A single request must not mix addresses from different segments.
static constexpr uint16_t BREAKPOINT_1 = 12100;
static constexpr uint16_t BREAKPOINT_2 = 12500;

// --- Structured response types ---

struct RegisterValue {
  uint16_t address;
  uint16_t value;
};

struct ParsedResponse {
  uint8_t function_code{0};
  bool is_error{false};
  uint8_t error_code{0};                    // Only valid when is_error == true
  std::vector<RegisterValue> registers;      // Only valid for successful read responses
};

// --- CRC ---

/// Standard Modbus CRC-16 (polynomial 0xA001, init 0xFFFF).
/// Returns 0xFFFF for null/empty input.
uint16_t crc16(const uint8_t *data, size_t len);

// --- Frame building (all return complete RTU frames with CRC) ---

/// Build func 0x41 request: read contiguous register ranges.
/// Each pair is (start_address, count). Caller must ensure all ranges
/// are within the same breakpoint segment.
/// Returns empty vector if ranges is empty.
std::vector<uint8_t> build_read_ranges_request(
    uint8_t slave_addr,
    const std::vector<std::pair<uint16_t, uint16_t>> &ranges);

/// Build func 0x42 request: read individual discontiguous registers.
/// Returns empty vector if addresses is empty or count > MAX_REGISTERS_PER_REQUEST.
std::vector<uint8_t> build_read_registers_request(
    uint8_t slave_addr,
    const uint16_t *addresses, size_t count);

/// Convenience overload for std::vector.
inline std::vector<uint8_t> build_read_registers_request(
    uint8_t slave_addr,
    const std::vector<uint16_t> &addresses) {
  return build_read_registers_request(slave_addr, addresses.data(), addresses.size());
}

/// Build func 0x03 request: standard read holding registers.
std::vector<uint8_t> build_read_holding_request(
    uint8_t slave_addr,
    uint16_t start_addr, uint16_t count);

/// Build func 0x06 request: write single register.
std::vector<uint8_t> build_write_single_request(
    uint8_t slave_addr,
    uint16_t address, uint16_t value);

/// Build func 0x43 request: write multiple discontiguous registers.
/// Each pair is (address, value). Returns empty vector if writes is empty.
std::vector<uint8_t> build_write_multi_request(
    uint8_t slave_addr,
    const std::vector<std::pair<uint16_t, uint16_t>> &writes);

// --- Frame validation ---

/// Validate CRC of a complete frame. Frame must be at least MIN_FRAME_SIZE bytes.
bool validate_frame_crc(const uint8_t *data, size_t len);

/// Check if a function code indicates an error response (high bit set).
inline bool is_error_response(uint8_t function_code) {
  return (function_code & ERROR_MASK) != 0;
}

/// Determine the expected total frame size from a partial receive buffer.
/// Returns the expected size if determinable, or 0 if more bytes are needed.
/// This centralizes frame-completeness logic that rw-wf duplicates in the hub.
///
/// For read responses (0x03, 0x41, 0x42): needs at least 3 bytes (addr + func + byte_count)
///   → expected = 3 + byte_count + 2 (CRC)
/// For write echo (0x06): always 8 bytes
/// For error response (func | 0x80): always 5 bytes
/// For write multi echo (0x43): needs at least 3 bytes → 3 + byte_count + 2
size_t expected_frame_size(const uint8_t *data, size_t available);

// --- Response parsing ---

/// Parse a complete frame into a structured response.
/// Validates CRC, extracts function code, handles error responses.
/// For read responses (0x42, 0x41), maps values back to expected_addresses.
/// For read holding (0x03), maps values to sequential addresses starting at start_address.
///
/// rw-wf's parse_register_values() returns raw uint16 values; the caller
/// must separately track expected_addresses and zip them together.
/// Our version takes the expected addresses and returns RegisterValue pairs.
ParsedResponse parse_frame(
    const uint8_t *frame, size_t frame_len,
    const uint16_t *expected_addresses = nullptr, size_t expected_count = 0);

/// Convenience overload for std::vector.
inline ParsedResponse parse_frame(
    const uint8_t *frame, size_t frame_len,
    const std::vector<uint16_t> &expected_addresses) {
  return parse_frame(frame, frame_len, expected_addresses.data(), expected_addresses.size());
}

// --- Breakpoint-aware helpers ---

/// Determine which breakpoint segment an address falls in.
/// Returns 0 (< 12100), 1 (12100–12499), or 2 (>= 12500).
inline uint8_t address_segment(uint16_t address) {
  if (address < BREAKPOINT_1) return 0;
  if (address < BREAKPOINT_2) return 1;
  return 2;
}

/// Split a list of addresses into per-segment groups.
/// Returns three vectors: [segment 0 addrs, segment 1 addrs, segment 2 addrs].
/// Each segment's addresses are sorted.
std::array<std::vector<uint16_t>, 3> split_by_segment(
    const std::vector<uint16_t> &addresses);

/// Merge sorted addresses into contiguous ranges where gaps <= max_gap.
/// Useful for converting individual addresses into efficient func 0x41 ranges.
/// Each returned pair is (start_address, count).
std::vector<std::pair<uint16_t, uint16_t>> merge_to_ranges(
    const std::vector<uint16_t> &sorted_addresses, uint16_t max_gap = 8);

}  // namespace protocol
}  // namespace waterfurnace_aurora
}  // namespace esphome
