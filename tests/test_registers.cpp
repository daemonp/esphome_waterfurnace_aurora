#include "catch_amalgamated.hpp"
#include "registers.h"

using namespace esphome::waterfurnace_aurora;

// ============================================================================
// Register Type Conversions
// ============================================================================

TEST_CASE("convert_register", "[registers][conversion]") {
  SECTION("UNSIGNED") {
    REQUIRE(convert_register(240, RegisterType::UNSIGNED) == 240.0f);
    REQUIRE(convert_register(0, RegisterType::UNSIGNED) == 0.0f);
    REQUIRE(convert_register(65535, RegisterType::UNSIGNED) == 65535.0f);
  }

  SECTION("SIGNED") {
    REQUIRE(convert_register(100, RegisterType::SIGNED) == 100.0f);
    // Negative: -100 as uint16 = 0xFF9C = 65436
    uint16_t neg100 = static_cast<uint16_t>(static_cast<int16_t>(-100));
    REQUIRE(convert_register(neg100, RegisterType::SIGNED) == -100.0f);
  }

  SECTION("TENTHS") {
    REQUIRE(convert_register(700, RegisterType::TENTHS) == Catch::Approx(70.0f));
    REQUIRE(convert_register(0, RegisterType::TENTHS) == 0.0f);

    SECTION("sentinel 999.9 -> NAN") {
      REQUIRE(std::isnan(convert_register(0x270F, RegisterType::TENTHS)));
    }
  }

  SECTION("SIGNED_TENTHS") {
    REQUIRE(convert_register(700, RegisterType::SIGNED_TENTHS) == Catch::Approx(70.0f));

    SECTION("negative values") {
      uint16_t neg = static_cast<uint16_t>(static_cast<int16_t>(-105));
      REQUIRE(convert_register(neg, RegisterType::SIGNED_TENTHS) == Catch::Approx(-10.5f));
    }

    SECTION("sentinel -999.9 -> NAN") {
      REQUIRE(std::isnan(convert_register(0xD8F1, RegisterType::SIGNED_TENTHS)));
    }

    SECTION("sentinel 999.9 -> NAN") {
      REQUIRE(std::isnan(convert_register(0x270F, RegisterType::SIGNED_TENTHS)));
    }
  }

  SECTION("HUNDREDTHS") {
    REQUIRE(convert_register(350, RegisterType::HUNDREDTHS) == Catch::Approx(3.5f));
    REQUIRE(convert_register(100, RegisterType::HUNDREDTHS) == Catch::Approx(1.0f));
  }

  SECTION("BOOLEAN") {
    REQUIRE(convert_register(0, RegisterType::BOOLEAN) == 0.0f);
    REQUIRE(convert_register(1, RegisterType::BOOLEAN) == 1.0f);
    REQUIRE(convert_register(42, RegisterType::BOOLEAN) == 1.0f);
    REQUIRE(convert_register(65535, RegisterType::BOOLEAN) == 1.0f);
  }
}

// ============================================================================
// Standalone Conversion Functions
// ============================================================================

TEST_CASE("to_signed_tenths", "[registers][conversion]") {
  REQUIRE(to_signed_tenths(700) == Catch::Approx(70.0f));

  uint16_t neg = static_cast<uint16_t>(static_cast<int16_t>(-200));
  REQUIRE(to_signed_tenths(neg) == Catch::Approx(-20.0f));
}

TEST_CASE("to_tenths", "[registers][conversion]") {
  REQUIRE(to_tenths(700) == Catch::Approx(70.0f));
  REQUIRE(to_tenths(0) == 0.0f);
}

TEST_CASE("32-bit assembly", "[registers][conversion]") {
  SECTION("to_uint32") {
    REQUIRE(to_uint32(0x0001, 0x0000) == 0x00010000);
    REQUIRE(to_uint32(0, 100) == 100);
    REQUIRE(to_uint32(1, 500) == 65536 + 500);
  }

  SECTION("to_int32 positive") {
    REQUIRE(to_int32(0, 100) == 100);
  }

  SECTION("to_int32 negative") {
    REQUIRE(to_int32(0xFFFF, 0xFFFF) == -1);
    REQUIRE(to_int32(0xFFFF, 0xFFFE) == -2);
  }
}

// ============================================================================
// registers_to_string
// ============================================================================

TEST_CASE("registers_to_string", "[registers]") {
  SECTION("ASCII conversion") {
    // 'A' = 0x41, 'B' = 0x42 -> register 0x4142
    std::vector<uint16_t> regs = {0x4142, 0x4344};
    REQUIRE(registers_to_string(regs) == "ABCD");
  }

  SECTION("strips trailing spaces") {
    std::vector<uint16_t> regs = {0x4142, 0x2020};  // "AB  "
    REQUIRE(registers_to_string(regs) == "AB");
  }

  SECTION("stops at null") {
    std::vector<uint16_t> regs = {0x4142, 0x0043};  // "AB" + null + 'C'
    REQUIRE(registers_to_string(regs) == "AB");
  }

  SECTION("empty input") {
    REQUIRE(registers_to_string({}).empty());
  }
}

// ============================================================================
// Fault Descriptions
// ============================================================================

TEST_CASE("get_fault_description", "[registers][fault]") {
  SECTION("known codes") {
    REQUIRE(std::string(get_fault_description(0)) == "No Fault");
    REQUIRE(std::string(get_fault_description(1)) == "Input Error");
    REQUIRE(std::string(get_fault_description(2)) == "High Pressure");
    REQUIRE(std::string(get_fault_description(99)) == "System Reset");
  }

  SECTION("unknown code") {
    REQUIRE(std::string(get_fault_description(50)) == "Unknown Fault");
    REQUIRE(std::string(get_fault_description(100)) == "Unknown Fault");
    REQUIRE(std::string(get_fault_description(255)) == "Unknown Fault");
  }
}

// ============================================================================
// String Lookups
// ============================================================================

TEST_CASE("get_hvac_mode_string", "[registers]") {
  REQUIRE(std::string(get_hvac_mode_string(HeatingMode::OFF)) == "Off");
  REQUIRE(std::string(get_hvac_mode_string(HeatingMode::AUTO)) == "Auto");
  REQUIRE(std::string(get_hvac_mode_string(HeatingMode::COOL)) == "Cool");
  REQUIRE(std::string(get_hvac_mode_string(HeatingMode::HEAT)) == "Heat");
  REQUIRE(std::string(get_hvac_mode_string(HeatingMode::EHEAT)) == "Emergency Heat");
}

TEST_CASE("get_fan_mode_string", "[registers]") {
  REQUIRE(std::string(get_fan_mode_string(FanMode::AUTO)) == "Auto");
  REQUIRE(std::string(get_fan_mode_string(FanMode::CONTINUOUS)) == "Continuous");
  REQUIRE(std::string(get_fan_mode_string(FanMode::INTERMITTENT)) == "Intermittent");
}

TEST_CASE("get_pump_type_string", "[registers]") {
  REQUIRE(std::string(get_pump_type_string(PumpType::OPEN_LOOP)) == "Open Loop");
  REQUIRE(std::string(get_pump_type_string(PumpType::VS_PUMP)) == "VS Pump");
  REQUIRE(std::string(get_pump_type_string(PumpType::OTHER)) == "Other");
}

// ============================================================================
// Bitmask-to-String
// ============================================================================

TEST_CASE("bitmask_to_string", "[registers][bitmask]") {
  SECTION("zero returns None") {
    REQUIRE(bitmask_to_string(0, VS_DERATE_BITS, VS_DERATE_BITS_COUNT) == "None");
  }

  SECTION("single flag") {
    REQUIRE(bitmask_to_string(VS_DERATE_DRIVE_OVER_TEMP, VS_DERATE_BITS, VS_DERATE_BITS_COUNT) == "Drive Over Temp");
  }

  SECTION("multiple flags") {
    uint16_t val = VS_DERATE_DRIVE_OVER_TEMP | VS_DERATE_OUTPUT_POWER_LIMIT;
    std::string result = bitmask_to_string(val, VS_DERATE_BITS, VS_DERATE_BITS_COUNT);
    REQUIRE(result.find("Drive Over Temp") != std::string::npos);
    REQUIRE(result.find("Output Power Limit") != std::string::npos);
    REQUIRE(result.find(", ") != std::string::npos);
  }
}

TEST_CASE("get_vs_derate_string", "[registers][bitmask]") {
  REQUIRE(get_vs_derate_string(0) == "None");
  REQUIRE(get_vs_derate_string(VS_DERATE_DRIVE_OVER_TEMP) == "Drive Over Temp");
}

TEST_CASE("get_vs_safe_mode_string", "[registers][bitmask]") {
  REQUIRE(get_vs_safe_mode_string(0) == "None");
  REQUIRE(get_vs_safe_mode_string(VS_SAFE_EEV_INDOOR_FAILED) == "EEV Indoor Failed");
}

TEST_CASE("get_vs_alarm_string", "[registers][bitmask]") {
  REQUIRE(get_vs_alarm_string(0, 0) == "None");
  REQUIRE(get_vs_alarm_string(0x8000, 0).find("Internal Error") != std::string::npos);
  REQUIRE(get_vs_alarm_string(0, 0x0001).find("Multi Safe Modes") != std::string::npos);
}

TEST_CASE("get_axb_inputs_string", "[registers]") {
  // Always includes Addr= and Acc2=
  std::string result = get_axb_inputs_string(0);
  REQUIRE(result.find("Addr=") != std::string::npos);
  REQUIRE(result.find("Acc2=") != std::string::npos);

  // SmartGrid flag
  result = get_axb_inputs_string(0x001);
  REQUIRE(result.find("SmartGrid") != std::string::npos);
}

// ============================================================================
// IZ2 Zone Extraction
// ============================================================================

TEST_CASE("iz2_extract_mode", "[registers][iz2]") {
  SECTION("standard modes") {
    // mode bits are at config2 >> 8, 3-bit mask
    REQUIRE(iz2_extract_mode(0x0000) == HeatingMode::OFF);
    REQUIRE(iz2_extract_mode(0x0100) == HeatingMode::AUTO);
    REQUIRE(iz2_extract_mode(0x0200) == HeatingMode::COOL);
    REQUIRE(iz2_extract_mode(0x0300) == HeatingMode::HEAT);
  }

  SECTION("E-Heat uses 3-bit mask (our fix over Ruby gem's 2-bit)") {
    REQUIRE(iz2_extract_mode(0x0400) == HeatingMode::EHEAT);
  }

  SECTION("values > 4 clamp to OFF") {
    REQUIRE(iz2_extract_mode(0x0500) == HeatingMode::OFF);
    REQUIRE(iz2_extract_mode(0x0700) == HeatingMode::OFF);
  }
}

TEST_CASE("iz2_extract_fan_mode", "[registers][iz2]") {
  REQUIRE(iz2_extract_fan_mode(0x0000) == FanMode::AUTO);
  REQUIRE(iz2_extract_fan_mode(0x0080) == FanMode::CONTINUOUS);
  REQUIRE(iz2_extract_fan_mode(0x0100) == FanMode::INTERMITTENT);
}

TEST_CASE("iz2_extract_cooling_setpoint", "[registers][iz2]") {
  // cooling_sp = ((config1 & 0x7e) >> 1) + 36
  // If bits 1-6 = 36 (decimal), sp = 36 + 36 = 72
  uint16_t config1 = 36 << 1;  // 0x48
  REQUIRE(iz2_extract_cooling_setpoint(config1) == Catch::Approx(72.0f));

  // Min: bits = 0 → 36
  REQUIRE(iz2_extract_cooling_setpoint(0x0000) == Catch::Approx(36.0f));
}

TEST_CASE("iz2_extract_heating_setpoint", "[registers][iz2]") {
  // heating_sp = ((carry << 5) | ((config2 & 0xf800) >> 11)) + 36
  // Test: carry=0, config2 upper 5 bits = 20 → sp = 20 + 36 = 56
  uint16_t config1 = 0x0000;  // carry = bit 0 = 0
  uint16_t config2 = 20 << 11;  // 0xA000
  REQUIRE(iz2_extract_heating_setpoint(config1, config2) == Catch::Approx(56.0f));

  // Test with carry bit
  config1 = 0x0001;  // carry = 1
  config2 = 0x0000;  // upper 5 bits = 0
  // sp = ((1 << 5) | 0) + 36 = 32 + 36 = 68
  REQUIRE(iz2_extract_heating_setpoint(config1, config2) == Catch::Approx(68.0f));
}

TEST_CASE("iz2_extract_damper_open", "[registers][iz2]") {
  REQUIRE_FALSE(iz2_extract_damper_open(0x0000));
  REQUIRE(iz2_extract_damper_open(0x0010));
}

TEST_CASE("iz2_extract_current_call", "[registers][iz2]") {
  // call = (config2 >> 1) & 0x7
  REQUIRE(iz2_extract_current_call(0x0000) == ZoneCall::STANDBY);
  REQUIRE(iz2_extract_current_call(0x0004) == ZoneCall::H1);  // (4 >> 1) & 7 = 2
  REQUIRE(iz2_extract_current_call(0x000A) == ZoneCall::C1);  // (10 >> 1) & 7 = 5
}

TEST_CASE("iz2_extract_fan_on_time", "[registers][iz2]") {
  // fan_on = ((config1 >> 9) & 0x7) * 5
  uint16_t config1 = 3 << 9;  // bits 9-11 = 3
  REQUIRE(iz2_extract_fan_on_time(config1) == 15);

  REQUIRE(iz2_extract_fan_on_time(0) == 0);
}

TEST_CASE("iz2_extract_fan_off_time", "[registers][iz2]") {
  // fan_off = (((config1 >> 12) & 0x7) + 1) * 5
  uint16_t config1 = 2 << 12;  // bits 12-14 = 2
  REQUIRE(iz2_extract_fan_off_time(config1) == 15);  // (2+1)*5

  REQUIRE(iz2_extract_fan_off_time(0) == 5);  // (0+1)*5
}

TEST_CASE("iz2_fan_desired", "[registers][iz2]") {
  REQUIRE(iz2_fan_desired(1) == 25);
  REQUIRE(iz2_fan_desired(2) == 40);
  REQUIRE(iz2_fan_desired(3) == 55);
  REQUIRE(iz2_fan_desired(4) == 70);
  REQUIRE(iz2_fan_desired(5) == 85);
  REQUIRE(iz2_fan_desired(6) == 100);
  REQUIRE(iz2_fan_desired(0) == 0);  // Fallback
}

// ============================================================================
// Register Map (flat sorted vector)
// ============================================================================

TEST_CASE("RegisterMap operations", "[registers][map]") {
  RegisterMap map;

  SECTION("insert and find") {
    reg_insert(map, 502, 700);
    const uint16_t *val = reg_find(map, 502);
    REQUIRE(val != nullptr);
    REQUIRE(*val == 700);
  }

  SECTION("find returns nullptr for missing") {
    REQUIRE(reg_find(map, 999) == nullptr);
  }

  SECTION("update existing key") {
    reg_insert(map, 502, 700);
    reg_insert(map, 502, 800);
    const uint16_t *val = reg_find(map, 502);
    REQUIRE(val != nullptr);
    REQUIRE(*val == 800);
  }

  SECTION("maintains sorted order") {
    reg_insert(map, 742, 100);
    reg_insert(map, 19, 200);
    reg_insert(map, 502, 300);

    REQUIRE(map.size() == 3);
    REQUIRE(map[0].first == 19);
    REQUIRE(map[1].first == 502);
    REQUIRE(map[2].first == 742);

    REQUIRE(*reg_find(map, 19) == 200);
    REQUIRE(*reg_find(map, 502) == 300);
    REQUIRE(*reg_find(map, 742) == 100);
  }
}
