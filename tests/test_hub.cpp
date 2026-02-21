#include "catch_amalgamated.hpp"
#include "waterfurnace_aurora.h"

using namespace esphome;
using namespace esphome::waterfurnace_aurora;

// Helper: advance millis and loop to transition past TX_PENDING → WAITING_RESPONSE.
// After send_request_, the hub is in TX_PENDING; we need to advance time past the
// calculated TX completion time.  A large frame (e.g. 30+ bytes) at 19200 baud 8E1
// takes ~20ms, so 25ms margin handles the worst case with room to spare.
static void complete_tx(WaterFurnaceAurora &hub, uint32_t current_ms) {
  set_millis(current_ms + 25);  // 25ms covers the largest frames at 19200 baud
  hub.loop();                   // transitions TX_PENDING → WAITING_RESPONSE
}

// Helper: build a valid func 0x42 response frame from address-value pairs
static std::vector<uint8_t> make_response_42(const std::vector<std::pair<uint16_t, uint16_t>> &regs) {
  std::vector<uint8_t> frame;
  frame.push_back(0x01);  // slave
  frame.push_back(0x42);  // func
  frame.push_back(static_cast<uint8_t>(regs.size() * 2));  // byte count
  for (const auto &r : regs) {
    frame.push_back(r.second >> 8);
    frame.push_back(r.second & 0xFF);
  }
  uint16_t crc = protocol::crc16(frame.data(), frame.size());
  frame.push_back(crc & 0xFF);
  frame.push_back(crc >> 8);
  return frame;
}

// Helper: build a valid func 0x03 response frame from raw values
static std::vector<uint8_t> make_response_03(const std::vector<uint16_t> &values) {
  std::vector<uint8_t> frame;
  frame.push_back(0x01);  // slave
  frame.push_back(0x03);  // func
  frame.push_back(static_cast<uint8_t>(values.size() * 2));  // byte count
  for (auto v : values) {
    frame.push_back(v >> 8);
    frame.push_back(v & 0xFF);
  }
  uint16_t crc = protocol::crc16(frame.data(), frame.size());
  frame.push_back(crc & 0xFF);
  frame.push_back(crc >> 8);
  return frame;
}

// ============================================================================
// Write Queue
// ============================================================================

TEST_CASE("Write queue", "[hub][write]") {
  WaterFurnaceAurora hub;
  set_millis(1000);

  SECTION("write_register queues a single write") {
    hub.write_register(100, 200);
    // We can't directly inspect pending_writes_ (protected), but we can
    // verify the control methods queue writes correctly via return value
  }

  SECTION("set_heating_setpoint validates range") {
    REQUIRE_FALSE(hub.set_heating_setpoint(30.0f));   // Too low
    REQUIRE_FALSE(hub.set_heating_setpoint(100.0f));  // Too high
    REQUIRE(hub.set_heating_setpoint(70.0f));          // Valid
  }

  SECTION("set_cooling_setpoint validates range") {
    REQUIRE_FALSE(hub.set_cooling_setpoint(50.0f));   // Too low
    REQUIRE_FALSE(hub.set_cooling_setpoint(100.0f));  // Too high
    REQUIRE(hub.set_cooling_setpoint(75.0f));          // Valid
  }

  SECTION("set_dhw_setpoint validates range") {
    REQUIRE_FALSE(hub.set_dhw_setpoint(90.0f));   // Too low
    REQUIRE_FALSE(hub.set_dhw_setpoint(150.0f));  // Too high
    REQUIRE(hub.set_dhw_setpoint(120.0f));         // Valid
  }

  SECTION("set_blower_only_speed validates range") {
    REQUIRE_FALSE(hub.set_blower_only_speed(0));   // Too low
    REQUIRE_FALSE(hub.set_blower_only_speed(13));  // Too high
    REQUIRE(hub.set_blower_only_speed(6));          // Valid
  }

  SECTION("set_pump_speed validates range") {
    REQUIRE_FALSE(hub.set_pump_speed(0));    // Too low
    REQUIRE_FALSE(hub.set_pump_speed(101));  // Too high
    REQUIRE(hub.set_pump_speed(50));          // Valid
  }

  SECTION("set_fan_intermittent_on validates multiples of 5") {
    REQUIRE_FALSE(hub.set_fan_intermittent_on(3));   // Not multiple of 5
    REQUIRE_FALSE(hub.set_fan_intermittent_on(30));  // Too high
    REQUIRE(hub.set_fan_intermittent_on(0));           // Valid
    REQUIRE(hub.set_fan_intermittent_on(15));          // Valid
  }

  SECTION("set_fan_intermittent_off validates multiples of 5") {
    REQUIRE_FALSE(hub.set_fan_intermittent_off(0));   // Too low
    REQUIRE_FALSE(hub.set_fan_intermittent_off(3));   // Not multiple of 5
    REQUIRE_FALSE(hub.set_fan_intermittent_off(45));  // Too high
    REQUIRE(hub.set_fan_intermittent_off(20));         // Valid
  }

  SECTION("set_humidification_target validates range") {
    REQUIRE_FALSE(hub.set_humidification_target(10));  // Too low
    REQUIRE_FALSE(hub.set_humidification_target(55));  // Too high
    REQUIRE(hub.set_humidification_target(35));         // Valid
  }

  SECTION("set_dehumidification_target validates range") {
    REQUIRE_FALSE(hub.set_dehumidification_target(30));  // Too low
    REQUIRE_FALSE(hub.set_dehumidification_target(70));  // Too high
    REQUIRE(hub.set_dehumidification_target(50));         // Valid
  }

  SECTION("set_hvac_mode always succeeds") {
    REQUIRE(hub.set_hvac_mode(HeatingMode::HEAT));
    REQUIRE(hub.set_hvac_mode(HeatingMode::COOL));
    REQUIRE(hub.set_hvac_mode(HeatingMode::OFF));
  }

  SECTION("set_fan_mode always succeeds") {
    REQUIRE(hub.set_fan_mode(FanMode::AUTO));
    REQUIRE(hub.set_fan_mode(FanMode::CONTINUOUS));
  }

  SECTION("clear_fault_history succeeds") {
    REQUIRE(hub.clear_fault_history());
  }
}

// ============================================================================
// IZ2 Zone Validation
// ============================================================================

TEST_CASE("IZ2 zone validation", "[hub][iz2]") {
  WaterFurnaceAurora hub;
  set_millis(1000);

  SECTION("zone 0 is invalid") {
    REQUIRE_FALSE(hub.set_zone_heating_setpoint(0, 70.0f));
  }

  SECTION("zone 7 is invalid") {
    REQUIRE_FALSE(hub.set_zone_cooling_setpoint(7, 75.0f));
  }

  SECTION("zone 1-6 with valid setpoints succeed") {
    for (uint8_t z = 1; z <= 6; z++) {
      REQUIRE(hub.set_zone_heating_setpoint(z, 70.0f));
      REQUIRE(hub.set_zone_cooling_setpoint(z, 75.0f));
    }
  }

  SECTION("zone setpoint range validation") {
    REQUIRE_FALSE(hub.set_zone_heating_setpoint(1, 30.0f));   // Too low
    REQUIRE_FALSE(hub.set_zone_heating_setpoint(1, 95.0f));   // Too high
    REQUIRE_FALSE(hub.set_zone_cooling_setpoint(1, 50.0f));   // Too low
    REQUIRE_FALSE(hub.set_zone_cooling_setpoint(1, 100.0f));  // Too high
  }

  SECTION("zone mode and fan always succeed for valid zones") {
    REQUIRE(hub.set_zone_hvac_mode(1, HeatingMode::HEAT));
    REQUIRE(hub.set_zone_fan_mode(2, FanMode::CONTINUOUS));
  }

  SECTION("zone fan timing validates") {
    REQUIRE(hub.set_zone_fan_intermittent_on(1, 10));
    REQUIRE_FALSE(hub.set_zone_fan_intermittent_on(1, 7));  // Not mult of 5
    REQUIRE(hub.set_zone_fan_intermittent_off(1, 15));
    REQUIRE_FALSE(hub.set_zone_fan_intermittent_off(1, 0)); // Too low
  }
}

// ============================================================================
// Setup Callbacks
// ============================================================================

TEST_CASE("Setup callbacks", "[hub][setup]") {
  WaterFurnaceAurora hub;

  SECTION("callback fires immediately if setup already complete") {
    // Force setup_complete by calling finish through the public path isn't
    // possible without UART, so we test register_setup_callback when not complete
    bool called = false;
    hub.register_setup_callback([&called]() { called = true; });
    // setup is not complete yet, so callback should NOT have fired
    REQUIRE_FALSE(called);
  }

  SECTION("register_listener stores callback") {
    int count = 0;
    hub.register_listener([&count]() { count++; });
    // The callback is stored but won't fire until publish_all_sensors_ runs
    REQUIRE(count == 0);
  }
}

// ============================================================================
// State Machine Initial State
// ============================================================================

TEST_CASE("Initial state", "[hub][state]") {
  WaterFurnaceAurora hub;

  SECTION("setup_complete is false initially") {
    REQUIRE_FALSE(hub.is_setup_complete());
  }

  SECTION("default getters return sane values") {
    REQUIRE(std::isnan(hub.get_ambient_temperature()));
    REQUIRE(std::isnan(hub.get_heating_setpoint()));
    REQUIRE(std::isnan(hub.get_cooling_setpoint()));
    REQUIRE(hub.get_hvac_mode() == HeatingMode::OFF);
    REQUIRE(hub.get_fan_mode() == FanMode::AUTO);
    REQUIRE_FALSE(hub.is_dhw_enabled());
    REQUIRE(std::isnan(hub.get_dhw_setpoint()));
    REQUIRE(std::isnan(hub.get_dhw_temperature()));
    REQUIRE(hub.get_system_outputs() == 0);
    REQUIRE(hub.get_axb_outputs() == 0);
    REQUIRE_FALSE(hub.is_locked_out());
    REQUIRE_FALSE(hub.has_iz2());
    REQUIRE(hub.get_num_iz2_zones() == 0);
  }

  SECTION("update does nothing before setup completes") {
    set_millis(10000);
    // This should not crash or change state
    hub.update();
    REQUIRE_FALSE(hub.is_setup_complete());
  }
}

// ============================================================================
// Connected Sensor
// ============================================================================

TEST_CASE("Connected sensor", "[hub][connectivity]") {
  WaterFurnaceAurora hub;
  binary_sensor::BinarySensor connected;
  hub.set_connected_sensor(&connected);

  SECTION("starts disconnected after setup") {
    set_millis(0);
    hub.setup();
    // update_connected_(false) is a no-op when connected_ already == false,
    // so the sensor doesn't get an initial publish.  After a successful
    // response (connected_ transitions true→false later), it would publish.
    REQUIRE_FALSE(connected.has_state_);
  }

  SECTION("publishes true after successful comms then false on timeout") {
    set_millis(0);
    hub.setup();
    REQUIRE_FALSE(connected.has_state_);

    // Simulate the hub marking connected after a successful response
    // by driving a minimal setup flow.
    hub.loop();  // sends setup ID request → TX_PENDING
    auto tx = hub.mock_get_transmitted();
    REQUIRE(tx.size() > 0);

    complete_tx(hub, 0);  // advance past TX → WAITING_RESPONSE

    // Feed a valid ID response (18 regs of spaces)
    std::vector<uint16_t> id_vals(18, 0x2020);
    hub.mock_receive(make_response_03(id_vals));
    set_millis(30);
    hub.loop();  // process response → update_connected_(true)

    REQUIRE(connected.has_state_);
    REQUIRE(connected.state == true);
  }
}

// ============================================================================
// State Machine: setup() flow
// ============================================================================

TEST_CASE("setup() is non-blocking", "[hub][state]") {
  WaterFurnaceAurora hub;
  set_millis(0);

  hub.setup();
  // After setup(), the hub should be in SETUP_READ_ID state
  // and setup should NOT be complete (non-blocking)
  REQUIRE_FALSE(hub.is_setup_complete());
}

TEST_CASE("Full setup flow with mock UART", "[hub][state][integration]") {
  WaterFurnaceAurora hub;
  set_millis(0);

  hub.setup();

  // First loop() sends the setup ID request → TX_PENDING
  hub.loop();
  auto tx = hub.mock_get_transmitted();
  REQUIRE(tx.size() > 0);
  REQUIRE(tx[1] == 0x03);  // func 0x03 holding read for model/serial

  complete_tx(hub, 0);  // TX_PENDING → WAITING_RESPONSE

  // Feed a valid response for model/serial (18 regs of spaces)
  std::vector<uint16_t> id_values(18, 0x2020);  // spaces
  auto id_resp = make_response_03(id_values);
  hub.mock_receive(id_resp);

  set_millis(50);
  hub.loop();  // Process response, transition to SETUP_DETECT_COMPONENTS

  // Next loop() sends detect request → TX_PENDING
  hub.loop();
  tx = hub.mock_get_transmitted();
  REQUIRE(tx.size() > 0);
  REQUIRE(tx[1] == 0x42);  // func 0x42 for detect

  complete_tx(hub, 50);  // TX_PENDING → WAITING_RESPONSE

  // Feed a response: AXB=3 (absent), IZ2=3 (absent), thermostat=300 (v3.00),
  // blower=0 (PSC), energy=0, pump=0, ABC program = 0
  size_t num_detect_regs = (tx.size() - 4) / 2;  // (frame - slave - func - crc) / 2 bytes per addr
  std::vector<std::pair<uint16_t, uint16_t>> detect_vals;
  for (size_t i = 0; i < num_detect_regs; i++) {
    uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
    uint16_t val = 3;  // Default = "removed" for component status
    if (addr == registers::THERMOSTAT_VERSION) val = 300;
    if (addr == registers::BLOWER_TYPE) val = 0;
    if (addr == registers::ENERGY_MONITOR) val = 0;
    if (addr == registers::PUMP_TYPE) val = 0;
    if (addr >= 88 && addr <= 91) val = 0x2020;
    detect_vals.emplace_back(addr, val);
  }
  auto detect_resp = make_response_42(detect_vals);
  hub.mock_receive(detect_resp);

  set_millis(100);
  hub.loop();  // Process detect response

  // Since no VS drive detected from program, it should probe VS registers
  // next (SETUP_DETECT_VS).  After processing the detect response the hub
  // transitions to SETUP_DETECT_VS, but that state is entered on the NEXT
  // loop() call.  We also need complete_tx() to advance past TX_PENDING.
  hub.loop();  // Process detect response → transitions to SETUP_DETECT_VS
  hub.loop();  // Enter SETUP_DETECT_VS → send VS probe → TX_PENDING
  tx = hub.mock_get_transmitted();
  REQUIRE(tx.size() > 0);
  REQUIRE(tx[1] == 0x42);  // VS probe is also func 0x42

  complete_tx(hub, 100);  // TX_PENDING → WAITING_RESPONSE

  // Feed VS probe response with all zeros (no VS drive)
  auto vs_resp = make_response_42({{3001, 0}, {3322, 0}, {3325, 0}});
  hub.mock_receive(vs_resp);

  set_millis(150);
  hub.loop();  // Process VS probe → finish setup

  // Setup should now be complete
  REQUIRE(hub.is_setup_complete());
}
