#include "catch_amalgamated.hpp"
#include "waterfurnace_aurora.h"

using namespace esphome;
using namespace esphome::waterfurnace_aurora;

// Helper: advance millis and loop to transition past TX_PENDING → WAITING_RESPONSE.
// After send_request_, the hub is in TX_PENDING; we need to advance time past the
// calculated TX completion time.  A 100-register poll frame (~204 bytes) at 19200 baud
// 8E1 takes ~120ms, so 150ms margin handles the worst case with room to spare.
static void complete_tx(WaterFurnaceAurora &hub, uint32_t current_ms) {
  set_millis(current_ms + 150);  // 150ms covers the largest frames at 19200 baud
  hub.loop();                    // transitions TX_PENDING → WAITING_RESPONSE
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

  SECTION("set_pump_manual_control queues correct writes") {
    REQUIRE(hub.set_pump_manual_control(true));   // enables manual
    REQUIRE(hub.set_pump_manual_control(false));  // disables manual (writes 0x7FFF)
  }

  SECTION("set_line_voltage_setting validates range") {
    REQUIRE_FALSE(hub.set_line_voltage_setting(89));    // Too low
    REQUIRE_FALSE(hub.set_line_voltage_setting(636));   // Too high
    REQUIRE(hub.set_line_voltage_setting(90));            // Min valid
    REQUIRE(hub.set_line_voltage_setting(635));           // Max valid
    REQUIRE(hub.set_line_voltage_setting(240));           // Typical US
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

  SECTION("set_zone_fan_mode optimistically updates zone data") {
    // Default should be AUTO
    REQUIRE(hub.get_zone_data(1).target_fan_mode == FanMode::AUTO);

    REQUIRE(hub.set_zone_fan_mode(1, FanMode::CONTINUOUS));
    REQUIRE(hub.get_zone_data(1).target_fan_mode == FanMode::CONTINUOUS);

    REQUIRE(hub.set_zone_fan_mode(1, FanMode::INTERMITTENT));
    REQUIRE(hub.get_zone_data(1).target_fan_mode == FanMode::INTERMITTENT);

    REQUIRE(hub.set_zone_fan_mode(1, FanMode::AUTO));
    REQUIRE(hub.get_zone_data(1).target_fan_mode == FanMode::AUTO);

    // Verify it works for different zones independently
    REQUIRE(hub.set_zone_fan_mode(2, FanMode::INTERMITTENT));
    REQUIRE(hub.get_zone_data(2).target_fan_mode == FanMode::INTERMITTENT);
    REQUIRE(hub.get_zone_data(1).target_fan_mode == FanMode::AUTO);  // zone 1 unchanged
  }

  SECTION("zone fan timing validates") {
    REQUIRE(hub.set_zone_fan_intermittent_on(1, 10));
    REQUIRE_FALSE(hub.set_zone_fan_intermittent_on(1, 7));  // Not mult of 5
    REQUIRE(hub.set_zone_fan_intermittent_off(1, 15));
    REQUIRE_FALSE(hub.set_zone_fan_intermittent_off(1, 0)); // Too low
  }
}

// ============================================================================
// Setup Failure & Re-Detection (Issue 17)
// ============================================================================

TEST_CASE("Setup failure and re-detection", "[hub][state][redetect]") {
  WaterFurnaceAurora hub;
  
  // Create a minimal sensor to verify normal operation restoration
  sensor::Sensor entering_water;
  hub.set_entering_water_temperature_sensor(&entering_water);
  
  set_millis(0);
  hub.setup();

  SECTION("setup timeout after MAX_SETUP_RETRIES sets needs_redetect") {
    // Loop through 5 timeout cycles
    for (int i = 0; i < 5; i++) {
      hub.loop();                  // sends ID request → TX_PENDING
      hub.mock_get_transmitted();  // drain tx
      complete_tx(hub, millis());  // WAITING_RESPONSE
      
      // Advance past timeout
      set_millis(millis() + 2100);
      hub.loop();  // handle_timeout_() -> increments retry count -> ERROR_BACKOFF
      
      if (i < 4) {
        // Advance past backoff to start next retry
        set_millis(millis() + 5000);
        hub.loop();  // exits ERROR_BACKOFF -> SETUP_READ_ID
      }
    }
    
    // On the 5th timeout (now), finish_setup_() is called with defaults
    REQUIRE(hub.is_setup_complete());
    REQUIRE(hub.needs_redetect());
    REQUIRE_FALSE(hub.get_axb_status()); // Default is false
  }

  SECTION("first successful poll after failed setup triggers re-detection") {
    // 1. Force state to failed setup
    // We can't easily drive the loop 5 times in a sub-section without copy-paste,
    // so we'll simulate the end state by using the public API where possible,
    // but here we really need to just drive it.
    for (int i = 0; i < 5; i++) {
      hub.loop(); hub.mock_get_transmitted(); complete_tx(hub, millis());
      set_millis(millis() + 2100); hub.loop();
      if (i < 4) { set_millis(millis() + 5000); hub.loop(); }
    }
    REQUIRE(hub.needs_redetect());

    // 2. Trigger a poll cycle
    set_millis(millis() + 100);
    hub.update(); // builds minimal fast-tier address list
    auto tx = hub.mock_get_transmitted();
    REQUIRE(tx.size() > 0);
    complete_tx(hub, millis());

    // 3. Feed a valid 0x42 response
    // Response just needs to correspond to the requested registers.
    // Since detection failed, it's the minimal set.
    size_t num_regs = (tx.size() - 4) / 2;
    std::vector<std::pair<uint16_t, uint16_t>> resp_vals;
    for (size_t i = 0; i < num_regs; i++) {
      uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
      resp_vals.emplace_back(addr, 0); // Value doesn't matter much for triggering re-detect
    }
    hub.mock_receive(make_response_42(resp_vals));

    // 4. Process response -> trigger re-detect
    set_millis(millis() + 50);
    hub.loop(); 
    
    // Should have transitioned back to setup
    REQUIRE_FALSE(hub.is_setup_complete());
    REQUIRE_FALSE(hub.needs_redetect()); // Flag cleared
    
    // Next loop should send ID request
    hub.loop();
    tx = hub.mock_get_transmitted();
    REQUIRE(tx.size() > 0);
    REQUIRE(tx[1] == 0x03); // func 0x03 ID request
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

// Helper: drive a hub through the full setup sequence (ID + detect + VS probe).
// Returns the hub in IDLE state with setup_complete_ == true.
// Callers can set overrides before calling this.
static void drive_setup(WaterFurnaceAurora &hub) {
  hub.setup();

  // Step 1: ID request (func 0x03)
  hub.loop();                  // sends ID request → TX_PENDING
  hub.mock_get_transmitted();  // drain tx buffer
  complete_tx(hub, 0);         // TX_PENDING → WAITING_RESPONSE
  std::vector<uint16_t> id_values(18, 0x2020);
  hub.mock_receive(make_response_03(id_values));
  set_millis(50);
  hub.loop();                  // processes ID → SETUP_DETECT_COMPONENTS

  // Step 2: Detect request (func 0x42)
  hub.loop();                  // sends detect → TX_PENDING
  auto tx = hub.mock_get_transmitted();
  size_t num_detect_regs = (tx.size() - 4) / 2;
  std::vector<std::pair<uint16_t, uint16_t>> detect_vals;
  for (size_t i = 0; i < num_detect_regs; i++) {
    uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
    uint16_t val = 3;
    if (addr == registers::THERMOSTAT_VERSION) val = 300;
    if (addr == registers::BLOWER_TYPE) val = 0;
    if (addr == registers::ENERGY_MONITOR) val = 0;
    if (addr == registers::PUMP_TYPE) val = 0;
    if (addr >= 88 && addr <= 91) val = 0x2020;
    detect_vals.emplace_back(addr, val);
  }
  complete_tx(hub, 50);
  hub.mock_receive(make_response_42(detect_vals));
  set_millis(100);
  hub.loop();  // processes detect → SETUP_DETECT_VS

  // Step 3: VS probe (no VS drive)
  hub.loop();                  // enters SETUP_DETECT_VS
  hub.loop();                  // sends VS probe → TX_PENDING
  hub.mock_get_transmitted();  // drain tx buffer
  complete_tx(hub, 100);
  hub.mock_receive(make_response_42({{3001, 0}, {3322, 0}, {3325, 0}}));
  set_millis(150);
  hub.loop();                  // processes VS probe → finish setup → IDLE
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

// ============================================================================
// IZ2 Demand Sensors
// ============================================================================

TEST_CASE("IZ2 demand sensors", "[hub][iz2][sensors]") {
  WaterFurnaceAurora hub;
  sensor::Sensor fan_demand;
  sensor::Sensor unit_demand;
  hub.set_iz2_fan_demand_sensor(&fan_demand);
  hub.set_iz2_unit_demand_sensor(&unit_demand);
  hub.set_has_iz2_override(true);
  hub.set_num_iz2_zones_override(2);

  set_millis(0);
  drive_setup(hub);
  REQUIRE(hub.is_setup_complete());

  SECTION("register 31005 value 0x0403 → fan=4, unit=3") {
    // Verify IZ2 is configured
    REQUIRE(hub.has_iz2());
    REQUIRE(hub.get_num_iz2_zones() == 2);
    
    // Trigger a poll cycle
    set_millis(200);
    hub.update();  // starts poll → TX_PENDING
    auto tx = hub.mock_get_transmitted();
    REQUIRE(tx.size() > 0);
    complete_tx(hub, 200);

    // Build response: all registers zero except IZ2_DEMAND = 0x0403
    size_t num_regs = (tx.size() - 4) / 2;
    std::vector<std::pair<uint16_t, uint16_t>> resp_vals;
    for (size_t i = 0; i < num_regs; i++) {
      uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
      uint16_t val = 0;
      if (addr == registers::IZ2_DEMAND) val = 0x0403;
      resp_vals.emplace_back(addr, val);
    }
    hub.mock_receive(make_response_42(resp_vals));
    set_millis(400);
    hub.loop();  // process response → publish sensors

    REQUIRE(fan_demand.has_state_);
    REQUIRE(fan_demand.state == 4.0f);
    REQUIRE(unit_demand.has_state_);
    REQUIRE(unit_demand.state == 3.0f);
  }

  SECTION("register 31005 value 0x0000 → both zero") {
    set_millis(200);
    hub.update();
    auto tx = hub.mock_get_transmitted();
    complete_tx(hub, 200);

    size_t num_regs = (tx.size() - 4) / 2;
    std::vector<std::pair<uint16_t, uint16_t>> resp_vals;
    for (size_t i = 0; i < num_regs; i++) {
      uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
      uint16_t val = 0;
      // IZ2_DEMAND stays 0
      resp_vals.emplace_back(addr, val);
    }
    hub.mock_receive(make_response_42(resp_vals));
    set_millis(250);
    hub.loop();

    REQUIRE(fan_demand.has_state_);
    REQUIRE(fan_demand.state == 0.0f);
    REQUIRE(unit_demand.has_state_);
    REQUIRE(unit_demand.state == 0.0f);
  }

  SECTION("dedup: identical values don't re-publish") {
    // First poll
    set_millis(200);
    hub.update();
    auto tx = hub.mock_get_transmitted();
    complete_tx(hub, 200);

    size_t num_regs = (tx.size() - 4) / 2;
    std::vector<std::pair<uint16_t, uint16_t>> resp_vals;
    for (size_t i = 0; i < num_regs; i++) {
      uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
      uint16_t val = 0;
      if (addr == registers::IZ2_DEMAND) val = 0x0503;
      resp_vals.emplace_back(addr, val);
    }
    hub.mock_receive(make_response_42(resp_vals));
    set_millis(250);
    hub.loop();

    REQUIRE(fan_demand.publish_count_ == 1);
    REQUIRE(unit_demand.publish_count_ == 1);
    REQUIRE(fan_demand.state == 5.0f);
    REQUIRE(unit_demand.state == 3.0f);

    // Second poll with same values
    set_millis(300);
    hub.update();
    tx = hub.mock_get_transmitted();
    complete_tx(hub, 300);

    // Rebuild response (same values)
    num_regs = (tx.size() - 4) / 2;
    resp_vals.clear();
    for (size_t i = 0; i < num_regs; i++) {
      uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
      uint16_t val = 0;
      if (addr == registers::IZ2_DEMAND) val = 0x0503;
      resp_vals.emplace_back(addr, val);
    }
    hub.mock_receive(make_response_42(resp_vals));
    set_millis(350);
    hub.loop();

    // Should NOT have published again
    REQUIRE(fan_demand.publish_count_ == 1);
    REQUIRE(unit_demand.publish_count_ == 1);
  }
}

// ============================================================================
// Leaving Air Temperature Fallback
// ============================================================================

TEST_CASE("Leaving air temperature on non-AWL systems", "[hub][sensors]") {
  WaterFurnaceAurora hub;
  sensor::Sensor leaving_air;
  hub.set_leaving_air_temperature_sensor(&leaving_air);

  set_millis(0);

  SECTION("non-AWL-AXB publishes NAN after setup") {
    // drive_setup creates a non-AWL-AXB system by default (AXB absent)
    drive_setup(hub);
    REQUIRE(hub.is_setup_complete());

    // finish_setup_() should have published NAN for leaving air
    REQUIRE(leaving_air.has_state_);
    REQUIRE(std::isnan(leaving_air.state));
  }

  SECTION("AWL-AXB does not publish NAN after setup") {
    // Override to have AXB present — need to set up with AXB detected.
    // drive_setup defaults to no AXB; we override before calling it.
    hub.set_has_axb_override(true);

    // Custom drive_setup with AXB v2.00 (awl_axb = true: version >= 2.0)
    hub.setup();

    // Step 1: ID request
    hub.loop();
    hub.mock_get_transmitted();
    complete_tx(hub, 0);
    std::vector<uint16_t> id_values(18, 0x2020);
    hub.mock_receive(make_response_03(id_values));
    set_millis(50);
    hub.loop();

    // Step 2: Detect request
    hub.loop();
    auto tx = hub.mock_get_transmitted();
    size_t num_detect_regs = (tx.size() - 4) / 2;
    std::vector<std::pair<uint16_t, uint16_t>> detect_vals;
    for (size_t i = 0; i < num_detect_regs; i++) {
      uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
      uint16_t val = 3;
      if (addr == registers::THERMOSTAT_VERSION) val = 300;
      if (addr == registers::AXB_VERSION) val = 200;  // AWL AXB v2.00
      if (addr == registers::BLOWER_TYPE) val = 0;
      if (addr == registers::ENERGY_MONITOR) val = 0;
      if (addr == registers::PUMP_TYPE) val = 0;
      if (addr >= 88 && addr <= 91) val = 0x2020;
      detect_vals.emplace_back(addr, val);
    }
    complete_tx(hub, 50);
    hub.mock_receive(make_response_42(detect_vals));
    set_millis(100);
    hub.loop();

    // Step 3: VS probe
    hub.loop();
    hub.loop();
    hub.mock_get_transmitted();
    complete_tx(hub, 100);
    hub.mock_receive(make_response_42({{3001, 0}, {3322, 0}, {3325, 0}}));
    set_millis(150);
    hub.loop();

    REQUIRE(hub.is_setup_complete());
    // AWL AXB detected — leaving air should NOT have been published as NAN
    REQUIRE_FALSE(leaving_air.has_state_);
  }
}

// ============================================================================
// Pressure Switch Polarity (field validation fix)
// ============================================================================

TEST_CASE("Pressure switch polarity", "[hub][sensors][binary]") {
  WaterFurnaceAurora hub;
  binary_sensor::BinarySensor lps;
  binary_sensor::BinarySensor hps;
  hub.set_low_pressure_switch_binary_sensor(&lps);
  hub.set_high_pressure_switch_binary_sensor(&hps);

  set_millis(0);
  drive_setup(hub);
  REQUIRE(hub.is_setup_complete());

  // Helper: run a poll cycle with a specific SYSTEM_STATUS value
  auto run_poll_with_status = [&](uint16_t status_val, uint32_t time_ms) {
    set_millis(time_ms);
    hub.update();
    auto tx = hub.mock_get_transmitted();
    REQUIRE(tx.size() > 0);
    complete_tx(hub, time_ms);

    size_t num_regs = (tx.size() - 4) / 2;
    std::vector<std::pair<uint16_t, uint16_t>> resp_vals;
    for (size_t i = 0; i < num_regs; i++) {
      uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
      uint16_t val = 0;
      if (addr == registers::SYSTEM_STATUS) val = status_val;
      resp_vals.emplace_back(addr, val);
    }
    hub.mock_receive(make_response_42(resp_vals));
    set_millis(time_ms + 50);
    hub.loop();
  };

  SECTION("bits set (closed) → no problem (false)") {
    // Both LPS (0x80) and HPS (0x100) bits set = switches closed = normal
    run_poll_with_status(STATUS_LPS | STATUS_HPS, 200);
    REQUIRE(lps.has_state_);
    REQUIRE(lps.state == false);  // not a problem
    REQUIRE(hps.has_state_);
    REQUIRE(hps.state == false);  // not a problem
  }

  SECTION("bits clear (open) → problem (true)") {
    // Both bits clear = switches open = fault condition
    run_poll_with_status(0x0000, 200);
    REQUIRE(lps.has_state_);
    REQUIRE(lps.state == true);  // problem
    REQUIRE(hps.has_state_);
    REQUIRE(hps.state == true);  // problem
  }

  SECTION("only LPS open → LPS problem, HPS ok") {
    // HPS bit set (0x100), LPS bit clear
    run_poll_with_status(STATUS_HPS, 200);
    REQUIRE(lps.state == true);   // LPS open = problem
    REQUIRE(hps.state == false);  // HPS closed = ok
  }

  SECTION("only HPS open → HPS problem, LPS ok") {
    // LPS bit set (0x80), HPS bit clear
    run_poll_with_status(STATUS_LPS, 200);
    REQUIRE(lps.state == false);  // LPS closed = ok
    REQUIRE(hps.state == true);   // HPS open = problem
  }
}

// ============================================================================
// Entering Air Temperature Zero-Suppression
// ============================================================================

TEST_CASE("Entering air temperature suppresses zero values", "[hub][sensors]") {
  WaterFurnaceAurora hub;
  sensor::Sensor entering_air;
  hub.set_entering_air_temperature_sensor(&entering_air);

  set_millis(0);
  drive_setup(hub);  // non-AWL system → reads register 567
  REQUIRE(hub.is_setup_complete());

  auto run_poll_with_entering_air = [&](uint16_t raw_val, uint32_t time_ms) {
    set_millis(time_ms);
    hub.update();
    auto tx = hub.mock_get_transmitted();
    REQUIRE(tx.size() > 0);
    complete_tx(hub, time_ms);

    size_t num_regs = (tx.size() - 4) / 2;
    std::vector<std::pair<uint16_t, uint16_t>> resp_vals;
    for (size_t i = 0; i < num_regs; i++) {
      uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
      uint16_t val = 0;
      if (addr == registers::ENTERING_AIR) val = raw_val;
      resp_vals.emplace_back(addr, val);
    }
    hub.mock_receive(make_response_42(resp_vals));
    set_millis(time_ms + 50);
    hub.loop();
  };

  SECTION("zero value is not published") {
    run_poll_with_entering_air(0, 200);
    // Zero should be suppressed — sensor should not have state
    REQUIRE_FALSE(entering_air.has_state_);
  }

  SECTION("non-zero value is published as signed tenths") {
    // 698 = 69.8°F
    run_poll_with_entering_air(698, 200);
    REQUIRE(entering_air.has_state_);
    REQUIRE(entering_air.state == Catch::Approx(69.8f));
  }

  SECTION("non-zero then zero does not overwrite") {
    // First: valid reading
    run_poll_with_entering_air(698, 200);
    REQUIRE(entering_air.state == Catch::Approx(69.8f));
    int count_after_first = entering_air.publish_count_;

    // Second: zero — should not publish, value stays at 69.8
    run_poll_with_entering_air(0, 300);
    REQUIRE(entering_air.publish_count_ == count_after_first);
    REQUIRE(entering_air.state == Catch::Approx(69.8f));
  }
}

// ============================================================================
// Outdoor Temperature Zero-Suppression
// ============================================================================

TEST_CASE("Outdoor temperature suppresses zero values", "[hub][sensors]") {
  WaterFurnaceAurora hub;
  sensor::Sensor outdoor_temp;
  hub.set_outdoor_temperature_sensor(&outdoor_temp);

  // Need AWL communicating to poll outdoor temp (register 742)
  // drive_setup creates thermostat v3.00 which satisfies awl_thermostat()
  set_millis(0);
  drive_setup(hub);
  REQUIRE(hub.is_setup_complete());

  auto run_poll_with_outdoor = [&](uint16_t raw_val, uint32_t time_ms) {
    set_millis(time_ms);
    hub.update();
    auto tx = hub.mock_get_transmitted();
    REQUIRE(tx.size() > 0);
    complete_tx(hub, time_ms);

    size_t num_regs = (tx.size() - 4) / 2;
    std::vector<std::pair<uint16_t, uint16_t>> resp_vals;
    for (size_t i = 0; i < num_regs; i++) {
      uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
      uint16_t val = 0;
      if (addr == registers::OUTDOOR_TEMP) val = raw_val;
      resp_vals.emplace_back(addr, val);
    }
    hub.mock_receive(make_response_42(resp_vals));
    set_millis(time_ms + 50);
    hub.loop();
  };

  SECTION("zero value is not published") {
    run_poll_with_outdoor(0, 200);
    REQUIRE_FALSE(outdoor_temp.has_state_);
  }

  SECTION("non-zero value is published") {
    // 350 = 35.0°F
    run_poll_with_outdoor(350, 200);
    REQUIRE(outdoor_temp.has_state_);
    REQUIRE(outdoor_temp.state == Catch::Approx(35.0f));
  }
}

// ============================================================================
// Approach Temperature Compressor Guard
// ============================================================================

TEST_CASE("Approach temperature requires compressor running", "[hub][sensors][derived]") {
  WaterFurnaceAurora hub;
  sensor::Sensor approach;
  hub.set_approach_temperature_sensor(&approach);

  // Need AXB for water temps, and refrigeration monitoring for sat condenser
  hub.set_has_axb_override(true);

  // Custom drive_setup with AXB + refrigeration monitoring
  set_millis(0);
  hub.setup();

  // Step 1: ID request
  hub.loop();
  hub.mock_get_transmitted();
  complete_tx(hub, 0);
  std::vector<uint16_t> id_values(18, 0x2020);
  hub.mock_receive(make_response_03(id_values));
  set_millis(50);
  hub.loop();

  // Step 2: Detect request — AXB present, energy monitor level 1 (refrigeration)
  hub.loop();
  auto tx = hub.mock_get_transmitted();
  size_t num_detect_regs = (tx.size() - 4) / 2;
  std::vector<std::pair<uint16_t, uint16_t>> detect_vals;
  for (size_t i = 0; i < num_detect_regs; i++) {
    uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
    uint16_t val = 3;
    if (addr == registers::THERMOSTAT_VERSION) val = 300;
    if (addr == registers::AXB_VERSION) val = 200;  // AWL AXB v2.00
    if (addr == registers::BLOWER_TYPE) val = 0;
    if (addr == registers::ENERGY_MONITOR) val = 1;  // refrigeration monitoring
    if (addr == registers::PUMP_TYPE) val = 0;
    if (addr >= 88 && addr <= 91) val = 0x2020;
    detect_vals.emplace_back(addr, val);
  }
  complete_tx(hub, 50);
  hub.mock_receive(make_response_42(detect_vals));
  set_millis(100);
  hub.loop();

  // Step 3: VS probe
  hub.loop();
  hub.loop();
  hub.mock_get_transmitted();
  complete_tx(hub, 100);
  hub.mock_receive(make_response_42({{3001, 0}, {3322, 0}, {3325, 0}}));
  set_millis(150);
  hub.loop();
  REQUIRE(hub.is_setup_complete());

  auto run_poll = [&](uint16_t outputs, uint16_t sat_cond, uint16_t leaving_air,
                      uint16_t ewt, uint32_t time_ms) {
    set_millis(time_ms);
    hub.update();
    auto tx2 = hub.mock_get_transmitted();
    REQUIRE(tx2.size() > 0);
    complete_tx(hub, time_ms);

    size_t num_regs = (tx2.size() - 4) / 2;
    std::vector<std::pair<uint16_t, uint16_t>> resp_vals;
    for (size_t i = 0; i < num_regs; i++) {
      uint16_t addr = (tx2[2 + i * 2] << 8) | tx2[3 + i * 2];
      uint16_t val = 0;
      if (addr == registers::SYSTEM_OUTPUTS) val = outputs;
      if (addr == registers::SATURATED_CONDENSER_TEMP) val = sat_cond;
      if (addr == registers::LEAVING_AIR) val = leaving_air;
      if (addr == registers::ENTERING_WATER) val = ewt;
      resp_vals.emplace_back(addr, val);
    }
    hub.mock_receive(make_response_42(resp_vals));
    set_millis(time_ms + 50);
    hub.loop();
  };

  SECTION("compressor off → publishes NAN") {
    // No CC/CC2 in outputs = compressor off
    // sat_cond=500 (50.0), leaving_air=720 (72.0), ewt=480 (48.0)
    run_poll(0x0000, 500, 720, 480, 200);
    REQUIRE(approach.has_state_);
    REQUIRE(std::isnan(approach.state));
  }

  SECTION("compressor on heating → publishes SatCond - LeavingAir") {
    // CC bit (0x01) set, no RV = heating mode
    // In heating, condenser is indoor air coil: sat_cond > leaving_air.
    // sat_cond=1050 (105.0°F), leaving_air=1020 (102.0°F) → approach = 105.0 - 102.0 = 3.0
    run_poll(OUTPUT_CC, 1050, 1020, 480, 200);
    REQUIRE(approach.has_state_);
    REQUIRE(approach.state == Catch::Approx(3.0f));
  }

  SECTION("compressor on cooling → publishes SatCond - EWT") {
    // CC bit (0x01) + RV bit (0x04) = cooling mode
    // sat_cond=500 (50.0°F), leaving_air=550 (55.0°F), ewt=480 (48.0°F)
    // approach = 50.0 - 48.0 = 2.0 (leaving_air unused in cooling)
    run_poll(OUTPUT_CC | OUTPUT_RV, 500, 550, 480, 200);
    REQUIRE(approach.has_state_);
    REQUIRE(approach.state == Catch::Approx(2.0f));
  }

  SECTION("transition from running to idle clears to NAN") {
    // First: compressor running in cooling → valid approach
    run_poll(OUTPUT_CC | OUTPUT_RV, 500, 550, 480, 200);
    REQUIRE_FALSE(std::isnan(approach.state));

    // Then: compressor off → NAN
    run_poll(0x0000, 500, 550, 480, 300);
    REQUIRE(std::isnan(approach.state));
  }
}

// ============================================================================
// VS Pump Manual Control
// ============================================================================

TEST_CASE("VS Pump manual control state tracking", "[hub][pump]") {
  WaterFurnaceAurora hub;
  set_millis(0);

  SECTION("initially not in manual control") {
    REQUIRE_FALSE(hub.is_pump_manual_control());
  }

  SECTION("set_pump_manual_control(true) sets state") {
    REQUIRE(hub.set_pump_manual_control(true));
    REQUIRE(hub.is_pump_manual_control());
  }

  SECTION("set_pump_manual_control(false) clears state") {
    hub.set_pump_manual_control(true);
    REQUIRE(hub.is_pump_manual_control());
    REQUIRE(hub.set_pump_manual_control(false));
    REQUIRE_FALSE(hub.is_pump_manual_control());
  }
}
