#include "catch_amalgamated.hpp"
#include "protocol.h"

using namespace esphome::waterfurnace_aurora::protocol;

// ============================================================================
// CRC-16
// ============================================================================

TEST_CASE("CRC16", "[protocol][crc]") {
  SECTION("known Modbus vector") {
    // Standard Modbus CRC test: slave 1, func 3, start 0, count 1
    uint8_t data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
    REQUIRE(crc16(data, sizeof(data)) == 0x0A84);
  }

  SECTION("null input returns 0xFFFF") {
    REQUIRE(crc16(nullptr, 0) == 0xFFFF);
  }

  SECTION("empty input returns 0xFFFF") {
    uint8_t data[] = {0};
    REQUIRE(crc16(data, 0) == 0xFFFF);
  }

  SECTION("single byte") {
    uint8_t data[] = {0x01};
    uint16_t result = crc16(data, 1);
    REQUIRE(result != 0xFFFF);  // Should be a valid CRC
  }

  SECTION("CRC is deterministic") {
    uint8_t data[] = {0x01, 0x42, 0x00, 0x19};
    uint16_t crc1 = crc16(data, sizeof(data));
    uint16_t crc2 = crc16(data, sizeof(data));
    REQUIRE(crc1 == crc2);
  }
}

// ============================================================================
// Frame Building
// ============================================================================

TEST_CASE("build_read_registers_request (func 0x42)", "[protocol][frame]") {
  SECTION("builds correct frame for single address") {
    auto frame = build_read_registers_request(1, {502});
    REQUIRE(frame.size() == 6);  // addr + func + 2 bytes addr + 2 CRC
    REQUIRE(frame[0] == 0x01);   // slave addr
    REQUIRE(frame[1] == 0x42);   // func code
    REQUIRE(frame[2] == 0x01);   // addr high
    REQUIRE(frame[3] == 0xF6);   // addr low (502 = 0x01F6)
    // CRC should validate
    REQUIRE(validate_frame_crc(frame.data(), frame.size()));
  }

  SECTION("builds correct frame for multiple addresses") {
    auto frame = build_read_registers_request(1, {19, 20, 30, 31});
    REQUIRE(frame.size() == 12);  // 2 + 4*2 + 2
    REQUIRE(frame[1] == 0x42);
    REQUIRE(validate_frame_crc(frame.data(), frame.size()));
  }

  SECTION("rejects empty addresses") {
    auto frame = build_read_registers_request(1, {});
    REQUIRE(frame.empty());
  }

  SECTION("rejects >100 addresses") {
    std::vector<uint16_t> too_many(101, 100);
    auto frame = build_read_registers_request(1, too_many);
    REQUIRE(frame.empty());
  }

  SECTION("accepts exactly 100 addresses") {
    std::vector<uint16_t> max_addrs(100, 100);
    auto frame = build_read_registers_request(1, max_addrs);
    REQUIRE_FALSE(frame.empty());
    REQUIRE(validate_frame_crc(frame.data(), frame.size()));
  }
}

TEST_CASE("build_read_ranges_request (func 0x41)", "[protocol][frame]") {
  SECTION("builds correct frame") {
    auto frame = build_read_ranges_request(1, {{92, 12}, {105, 5}});
    REQUIRE(frame.size() == 12);  // 2 + 2*4 + 2
    REQUIRE(frame[0] == 0x01);
    REQUIRE(frame[1] == 0x41);
    REQUIRE(validate_frame_crc(frame.data(), frame.size()));
  }

  SECTION("rejects empty ranges") {
    auto frame = build_read_ranges_request(1, {});
    REQUIRE(frame.empty());
  }
}

TEST_CASE("build_read_holding_request (func 0x03)", "[protocol][frame]") {
  SECTION("builds correct 8-byte frame") {
    auto frame = build_read_holding_request(1, 92, 12);
    REQUIRE(frame.size() == 8);
    REQUIRE(frame[0] == 0x01);
    REQUIRE(frame[1] == 0x03);
    REQUIRE(frame[2] == 0x00);   // start_addr high
    REQUIRE(frame[3] == 0x5C);   // start_addr low (92 = 0x5C)
    REQUIRE(frame[4] == 0x00);   // count high
    REQUIRE(frame[5] == 0x0C);   // count low (12 = 0x0C)
    REQUIRE(validate_frame_crc(frame.data(), frame.size()));
  }
}

TEST_CASE("build_write_single_request (func 0x06)", "[protocol][frame]") {
  SECTION("builds correct 8-byte frame") {
    auto frame = build_write_single_request(1, 12619, 700);
    REQUIRE(frame.size() == 8);
    REQUIRE(frame[0] == 0x01);
    REQUIRE(frame[1] == 0x06);
    REQUIRE(frame[2] == 0x31);   // 12619 >> 8 = 0x31
    REQUIRE(frame[3] == 0x4B);   // 12619 & 0xFF = 0x4B
    REQUIRE(frame[4] == 0x02);   // 700 >> 8
    REQUIRE(frame[5] == 0xBC);   // 700 & 0xFF
    REQUIRE(validate_frame_crc(frame.data(), frame.size()));
  }
}

TEST_CASE("build_write_multi_request (func 0x43)", "[protocol][frame]") {
  SECTION("builds correct frame for batch write") {
    auto frame = build_write_multi_request(1, {{12619, 700}, {12620, 750}});
    // 3 (header) + 2*4 (writes) + 2 (CRC) = 13
    REQUIRE(frame.size() == 13);
    REQUIRE(frame[0] == 0x01);
    REQUIRE(frame[1] == 0x43);
    REQUIRE(frame[2] == 8);      // byte_count = 2 writes * 4 bytes each
    REQUIRE(validate_frame_crc(frame.data(), frame.size()));
  }

  SECTION("rejects empty writes") {
    auto frame = build_write_multi_request(1, {});
    REQUIRE(frame.empty());
  }
}

// ============================================================================
// Frame Validation
// ============================================================================

TEST_CASE("validate_frame_crc", "[protocol][crc]") {
  SECTION("validates correct frame") {
    auto frame = build_write_single_request(1, 100, 200);
    REQUIRE(validate_frame_crc(frame.data(), frame.size()));
  }

  SECTION("rejects corrupted frame") {
    auto frame = build_write_single_request(1, 100, 200);
    frame[3] ^= 0xFF;  // Corrupt one byte
    REQUIRE_FALSE(validate_frame_crc(frame.data(), frame.size()));
  }

  SECTION("rejects null") {
    REQUIRE_FALSE(validate_frame_crc(nullptr, 0));
  }

  SECTION("rejects too short") {
    uint8_t data[] = {0x01, 0x03};
    REQUIRE_FALSE(validate_frame_crc(data, 2));
  }
}

// ============================================================================
// Expected Frame Size
// ============================================================================

TEST_CASE("expected_frame_size", "[protocol]") {
  SECTION("needs at least 2 bytes") {
    uint8_t data[] = {0x01};
    REQUIRE(expected_frame_size(data, 1) == 0);
  }

  SECTION("write single echo is always 8 bytes") {
    uint8_t data[] = {0x01, 0x06};
    REQUIRE(expected_frame_size(data, 2) == 8);
  }

  SECTION("error response is always 5 bytes") {
    uint8_t data[] = {0x01, 0x82};  // func 0x02 | 0x80
    REQUIRE(expected_frame_size(data, 2) == 5);
  }

  SECTION("read response needs byte_count (3 bytes)") {
    uint8_t data[] = {0x01, 0x42};
    REQUIRE(expected_frame_size(data, 2) == 0);  // Need more bytes
  }

  SECTION("read response with byte_count") {
    uint8_t data[] = {0x01, 0x42, 0x04};  // 4 bytes of data
    REQUIRE(expected_frame_size(data, 3) == 9);  // 3 + 4 + 2
  }

  SECTION("func 0x41 response") {
    uint8_t data[] = {0x01, 0x41, 0x18};  // 24 bytes of data
    REQUIRE(expected_frame_size(data, 3) == 29);  // 3 + 24 + 2
  }

  SECTION("func 0x03 response") {
    uint8_t data[] = {0x01, 0x03, 0x18};
    REQUIRE(expected_frame_size(data, 3) == 29);
  }
}

// ============================================================================
// Response Parsing
// ============================================================================

TEST_CASE("parse_frame", "[protocol][parse]") {
  SECTION("func 0x42 response maps values to addresses") {
    // Build a fake valid response: slave=1, func=0x42, byte_count=4, two values
    std::vector<uint8_t> frame = {0x01, 0x42, 0x04, 0x00, 0x64, 0x01, 0xF4};
    // Append CRC
    uint16_t crc = crc16(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);
    frame.push_back(crc >> 8);

    std::vector<uint16_t> expected = {502, 742};
    auto resp = parse_frame(frame.data(), frame.size(), expected);

    REQUIRE_FALSE(resp.is_error);
    REQUIRE(resp.function_code == 0x42);
    REQUIRE(resp.registers.size() == 2);
    REQUIRE(resp.registers[0].address == 502);
    REQUIRE(resp.registers[0].value == 0x0064);  // 100
    REQUIRE(resp.registers[1].address == 742);
    REQUIRE(resp.registers[1].value == 0x01F4);  // 500
  }

  SECTION("CRC mismatch returns error") {
    uint8_t frame[] = {0x01, 0x42, 0x02, 0x00, 0x64, 0xAA, 0xBB};  // Bad CRC
    auto resp = parse_frame(frame, sizeof(frame));
    REQUIRE(resp.is_error);
    REQUIRE(resp.error_code == 0xFE);  // CRC mismatch
  }

  SECTION("error response has is_error flag and code") {
    // Error response: slave=1, func=0xC2 (0x42|0x80), error_code=2
    std::vector<uint8_t> frame = {0x01, 0xC2, 0x02};
    uint16_t crc = crc16(frame.data(), frame.size());
    frame.push_back(crc & 0xFF);
    frame.push_back(crc >> 8);

    auto resp = parse_frame(frame.data(), frame.size());
    REQUIRE(resp.is_error);
    REQUIRE(resp.error_code == 0x02);
  }

  SECTION("write single echo has no register data") {
    auto frame = build_write_single_request(1, 12619, 700);
    auto resp = parse_frame(frame.data(), frame.size());
    REQUIRE_FALSE(resp.is_error);
    REQUIRE(resp.function_code == 0x06);
    REQUIRE(resp.registers.empty());
  }

  SECTION("frame too short returns error") {
    uint8_t frame[] = {0x01};
    auto resp = parse_frame(frame, 1);
    REQUIRE(resp.is_error);
  }

  SECTION("null frame returns error") {
    auto resp = parse_frame(nullptr, 0);
    REQUIRE(resp.is_error);
  }
}

// ============================================================================
// Breakpoint Helpers
// ============================================================================

TEST_CASE("address_segment", "[protocol][breakpoint]") {
  SECTION("segment 0: below 12100") {
    REQUIRE(address_segment(0) == 0);
    REQUIRE(address_segment(502) == 0);
    REQUIRE(address_segment(742) == 0);
    REQUIRE(address_segment(3001) == 0);
    REQUIRE(address_segment(3906) == 0);
    REQUIRE(address_segment(12099) == 0);
  }

  SECTION("segment 1: 12100-12499") {
    REQUIRE(address_segment(12100) == 1);
    REQUIRE(address_segment(12309) == 1);
    REQUIRE(address_segment(12499) == 1);
  }

  SECTION("segment 2: >= 12500") {
    REQUIRE(address_segment(12500) == 2);
    REQUIRE(address_segment(12619) == 2);
    REQUIRE(address_segment(21114) == 2);
    REQUIRE(address_segment(31003) == 2);
  }
}

TEST_CASE("split_by_segment", "[protocol][breakpoint]") {
  SECTION("splits mixed addresses into three groups") {
    std::vector<uint16_t> addrs = {502, 12309, 31003, 19, 12006, 21114, 742};
    auto segments = split_by_segment(addrs);

    // Segment 0: 19, 502, 742, 12006 (all < 12100)
    REQUIRE(segments[0].size() == 4);
    REQUIRE(segments[0][0] == 19);
    REQUIRE(segments[0][1] == 502);
    REQUIRE(segments[0][2] == 742);
    REQUIRE(segments[0][3] == 12006);

    // Segment 1: 12309 (12100-12499)
    REQUIRE(segments[1].size() == 1);
    REQUIRE(segments[1][0] == 12309);

    // Segment 2: 21114, 31003 (>= 12500)
    REQUIRE(segments[2].size() == 2);
    REQUIRE(segments[2][0] == 21114);
    REQUIRE(segments[2][1] == 31003);
  }

  SECTION("empty input produces empty segments") {
    auto segments = split_by_segment({});
    REQUIRE(segments[0].empty());
    REQUIRE(segments[1].empty());
    REQUIRE(segments[2].empty());
  }
}

TEST_CASE("merge_to_ranges", "[protocol][breakpoint]") {
  SECTION("adjacent addresses merge into one range") {
    auto ranges = merge_to_ranges({19, 20, 21, 22});
    REQUIRE(ranges.size() == 1);
    REQUIRE(ranges[0].first == 19);
    REQUIRE(ranges[0].second == 4);  // count
  }

  SECTION("gap within max_gap merges") {
    auto ranges = merge_to_ranges({19, 20, 28, 29}, 8);
    REQUIRE(ranges.size() == 1);
    REQUIRE(ranges[0].first == 19);
    REQUIRE(ranges[0].second == 11);  // 19 to 29 = 11 registers
  }

  SECTION("gap exceeding max_gap splits") {
    auto ranges = merge_to_ranges({19, 20, 100, 101}, 8);
    REQUIRE(ranges.size() == 2);
    REQUIRE(ranges[0].first == 19);
    REQUIRE(ranges[0].second == 2);
    REQUIRE(ranges[1].first == 100);
    REQUIRE(ranges[1].second == 2);
  }

  SECTION("single address") {
    auto ranges = merge_to_ranges({502});
    REQUIRE(ranges.size() == 1);
    REQUIRE(ranges[0].first == 502);
    REQUIRE(ranges[0].second == 1);
  }

  SECTION("empty input") {
    auto ranges = merge_to_ranges({});
    REQUIRE(ranges.empty());
  }
}

// ============================================================================
// is_error_response
// ============================================================================

TEST_CASE("is_error_response", "[protocol]") {
  REQUIRE(is_error_response(0x82));
  REQUIRE(is_error_response(0xC1));
  REQUIRE_FALSE(is_error_response(0x42));
  REQUIRE_FALSE(is_error_response(0x03));
  REQUIRE_FALSE(is_error_response(0x06));
}
