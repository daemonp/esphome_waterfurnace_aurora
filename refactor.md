# WaterFurnace Aurora ESPHome Component — Refactoring Plan v2

**Date:** 2026-02-21
**Scope:** All files under `components/waterfurnace_aurora/`, plus YAML configs
**Reference:** [waterfurnace_aurora Ruby gem](https://github.com/ccutrer/waterfurnace_aurora) v1.6.0 by ccutrer
**Comparative:** [espforge/esphome-waterfurnace](https://github.com/espforge/esphome-waterfurnace) (rw-wf) by Ryan Wagoner
**Prior Work:** v1 refactoring plan (Phases 1–3 and most of Phase 4 completed)

---

## Table of Contents

1. [Current State Summary](#1-current-state-summary)
2. [Architecture Comparison with rw-wf](#2-architecture-comparison-with-rw-wf)
3. [Refactoring Plan](#3-refactoring-plan)
   - [Phase 1: Protocol Separation](#phase-1-protocol-separation)
   - [Phase 2: Register Definitions Module](#phase-2-register-definitions-module)
   - [Phase 3: Async State Machine](#phase-3-async-state-machine)
   - [Phase 4: Testing Infrastructure](#phase-4-testing-infrastructure)
   - [Phase 5: Developer & User Experience](#phase-5-developer--user-experience)
   - [Phase 6: Feature Parity Additions](#phase-6-feature-parity-additions)
4. [What NOT to Change](#4-what-not-to-change)

---

## 1. Current State Summary

### Completed from v1 Plan

All of Phases 1–3 and most of Phase 4 from the original refactoring plan have been implemented:

| Item | Status |
|------|--------|
| NAN initialization for all temps | ✅ Done |
| Response validation in `read_holding_registers()` | ✅ Done |
| Named register constants for magic numbers | ✅ Done (`CLEAR_FAULT_HISTORY`, `CLEAR_FAULT_MAGIC`, `FAN_INTERMITTENT_ON/OFF_WRITE`, `ACTIVE_DEHUMIDIFY`) |
| Bounds check `get_zone_data()` | ✅ Done (`validate_zone_number()` extracted, used in 6 places) |
| Response buffer cap at 512 bytes | ✅ Done |
| `status_set_error()` on detection failure | ✅ Done |
| Sub-entity `setup_priority::PROCESSOR` | ✅ Done (all 6 sub-entities) |
| Complete `dump_config()` | ✅ Done (logs hardware details, blower type, pump type, energy level, AWL versions) |
| `reserve()` on string builders | ✅ Done (7 calls) |
| Publish only on change | ✅ Done (`publish_text_if_changed()`, observer callbacks) |
| AWL version checking (P-1) | ✅ Done (`awl_axb()`, `awl_thermostat()`, `awl_iz2()`, `awl_communicating()`) |
| Blower type detection (P-2) | ✅ Done (`BlowerType` enum class, register 404) |
| Pump type detection (P-3) | ✅ Done (`PumpType` enum, register 413, `is_vs_pump()`) |
| Energy monitor level detection (P-5) | ✅ Done (`energy_monitor_level_`, `refrigeration_monitoring()` / `energy_monitoring()`) |
| Extract `send_and_receive()` | ✅ Done (single method for all 4 Modbus operations) |
| Extract `validate_zone_number()` | ✅ Done (used in 6 places) |
| Extract climate mode mapping | ✅ Done (`aurora_climate_utils.h`) |
| Extract `publish_if_found()` helpers | ✅ Done (`publish_sensor()`, `publish_sensor_tenths()`, `publish_sensor_signed_tenths()`, `publish_text_if_changed()`) |
| IZ2 humidistat support (P-4) | ✅ Done |
| Replace `std::map` with flat register store (C-2) | ✅ Done (`RegisterMap` flat sorted vector with binary search) |
| Observer pattern for sub-entities (M-1) | ✅ Done (`register_listener()` + callback vector) |
| Convert enums to `enum class` (H-4) | ✅ Done (all 8 enums) |
| VS Drive telemetry sensors (P-6) | ✅ Done (`VS_FAN_SPEED`, `VS_AMBIENT_TEMP`, `VS_SAT_EVAP_DISCHARGE_TEMP`) |

### Remaining from v1 Plan

| Item | Status | Notes |
|------|--------|-------|
| Non-blocking Modbus state machine (C-3 / Phase 4.4) | ❌ Not done | The single biggest remaining architectural issue. Modbus I/O is synchronous/blocking. |
| Phase 5 feature additions | Partially done | Some sensors added; remaining are incremental |

### Current Codebase Stats

| File | Lines | Role |
|------|------:|------|
| `waterfurnace_aurora.h` | 852 | Hub class, register constants, enums, structs |
| `waterfurnace_aurora.cpp` | 2,034 | Hub implementation: Modbus I/O, detection, polling, controls |
| `climate/aurora_climate.cpp` | 192 | Main thermostat |
| `climate/aurora_iz2_climate.cpp` | 195 | IZ2 zone thermostat |
| `climate/aurora_climate_utils.h` | 91 | Shared mode mapping utilities |
| `number/aurora_dhw_number.cpp` | 117 | Number controls (11 entities via type dispatch) |
| `switch/aurora_dhw_switch.cpp` | 44 | DHW switch |
| `button/aurora_button.cpp` | 29 | Clear fault button |
| Various `__init__.py` | ~1,200 | Python config/codegen |
| **Total** | ~4,800 | |

---

## 2. Architecture Comparison with rw-wf

A side-by-side comparison of our project against espforge/esphome-waterfurnace (rw-wf), identifying patterns worth adopting:

### Communication Model

| Aspect | Ours (current) | rw-wf |
|--------|---------------|-------|
| I/O model | **Synchronous/blocking** — `update()` → `send_and_receive()` blocks waiting for response | **Async state machine** — `loop()` drives states: `SETUP_READ_ID → SETUP_DETECT_COMPONENTS → IDLE → WAITING_RESPONSE → ERROR_BACKOFF` |
| Blocking budget | Up to **4.5 seconds** worst case (all retries fail) | **Zero blocking** — bytes collected incrementally in `loop()`, response parsed when complete |
| Error recovery | Retry with `delay(50)` between attempts | Error backoff state with configurable timeout (5s), then resume |

**Verdict:** The async state machine is the single most impactful architectural improvement. It eliminates ESPHome `loop()` blocking warnings and prevents watchdog issues when hardware is disconnected.

### Protocol Layer

| Aspect | Ours (current) | rw-wf |
|--------|---------------|-------|
| Protocol code | Embedded in `waterfurnace_aurora.cpp` (mixed with hub logic) | **Separated `protocol.h/cpp`** (143 lines) — pure functions for CRC, frame building, parsing, validation |
| Register definitions | Named constants in hub header namespace | **Dedicated `registers.h`** (470 lines) — registers, enums, bitmasks, IZ2 helpers, fault table, capability enum, conversions |
| Testability | Not independently testable (coupled to UART, ESPHome framework) | Protocol layer is **fully unit-testable** (no ESPHome dependencies) |

**Verdict:** Separating protocol and register logic into standalone modules enables unit testing and reduces the hub class size by ~40%.

### Polling Strategy

| Aspect | Ours (current) | rw-wf |
|--------|---------------|-------|
| Groups | Three-tier adaptive (fast 5s, medium 30s, slow 5min) based on data volatility | Dynamic poll groups built from registered listeners at setup, segmented by protocol breakpoints |
| Register selection | Hub builds hardcoded address lists with conditional includes | **Listeners declare capabilities** via `RegisterCapability` enum; hub filters at group-build time |
| What gets polled | All registers in a tier, whether entities exist or not | **Only registers with active listeners** |

**Verdict:** Our three-tier approach is fine for data freshness semantics, but the listener-driven approach means we never poll registers nobody is listening to. Consider hybrid: keep tiered timing but only include addresses with registered listeners.

### Write Handling

| Aspect | Ours (current) | rw-wf |
|--------|---------------|-------|
| Write execution | Direct `write_holding_register()` — blocking, immediate | **Queued** to `pending_writes_` vector, sent non-blockingly via func 67 batch write |
| Write function code | Func 6 (write single register) — one request per write | **Func 67** (write multiple) — batch all pending writes in one request |
| Write cooldowns | None — read-backs may revert optimistic UI updates | **10-second cooldown** per write category (mode, setpoints, fan) — ignores stale read-backs |

**Verdict:** Both the write queue and cooldown mechanism are valuable. The cooldown prevents the "UI flicker" problem where an optimistic state update gets temporarily reverted by a stale read-back.

### Connectivity Monitoring

| Aspect | Ours (current) | rw-wf |
|--------|---------------|-------|
| Connected sensor | None | **Built-in `connected` binary sensor** with configurable timeout (default 30s) |
| Stale data handling | None — old values persist indefinitely | **Cache eviction** — expected addresses erased from register cache on timeout |

**Verdict:** The connected sensor is simple, high-value, and gives users visibility into communication health in Home Assistant.

### Developer Experience

| Aspect | Ours (current) | rw-wf |
|--------|---------------|-------|
| Tests | None | 38 unit tests (gtest) + 36 Docker integration tests against Ruby gem mock server |
| CI/CD | None | 3 GitHub Actions workflows (CI, firmware publish, pages publish) |
| Installation | `packages:` from GitHub | **One-click web install** via ESP Web Tools + Improv Serial + ESPHome Dashboard import |
| Release tooling | Manual | `release.sh` script with semantic versioning, tagging, GitHub Release creation |
| Web UI | Optional web server | Web server v3 with **sorting groups** for organized entity display |
| HA API services | None | `write_register` custom service for direct register writes from HA |

**Verdict:** Testing infrastructure is the most impactful DX improvement. Web install and release tooling are nice-to-haves.

### Our Unique Strengths (Keep These)

Features we have that rw-wf does not:

| Feature | Details |
|---------|---------|
| **11 number controls** | DHW setpoint, blower speeds (4), pump speeds (2), fan timing (2), humidity targets (2) |
| **Derived sensors** | COP, water delta-T, approach temperature — computed on-device |
| **Richer text sensors** | VS derate/safe mode/alarm status, humidifier/dehumidifier modes, pump type |
| **Fault history** | Button to clear + text sensor with full history from registers 601-699 |
| **Broader hardware support** | Any ESP32/ESP8266 + any MAX485 module (not locked to one board) |
| **Three-tier adaptive polling** | Fast (5s) for real-time, medium (30s) for config, slow (5min) for fault history |
| **Memory-optimized register store** | Flat sorted vector vs `std::map` — saves ~4KB heap fragmentation |

---

## 3. Refactoring Plan

### Phase 1: Protocol Separation

**Goal:** Extract Modbus protocol logic into a standalone, unit-testable module — improving on rw-wf's approach by adding request validation, breakpoint-aware batching, and structured response parsing.
**Effort:** 3–4 hours | **Risk:** Low | **Impact:** High (enables testing, reduces hub class by ~300 lines)

#### Where rw-wf falls short

rw-wf's `protocol.cpp` (143 lines) is intentionally minimal — just CRC, frame building, and byte parsing. The hub is left to:
- Split requests across protocol breakpoints (12100/12500) itself
- Track which addresses belong in which response
- Handle response/request address correlation manually
- No request validation (you can build an oversized request and only find out at the device)

Our protocol module will be **smarter** — it knows about WaterFurnace protocol constraints and does more work so the hub doesn't have to.

#### 1.1 — Create `protocol.h` with structured types and validated builders

```cpp
#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
#include <utility>

namespace esphome {
namespace waterfurnace_aurora {

// --- Protocol constants ---
static constexpr uint8_t FUNC_READ_RANGES = 0x41;
static constexpr uint8_t FUNC_READ_REGISTERS = 0x42;
static constexpr uint8_t FUNC_READ_HOLDING = 0x03;
static constexpr uint8_t FUNC_WRITE_SINGLE = 0x06;
static constexpr uint8_t FUNC_WRITE_MULTI = 0x43;  // Func 67 batch write
static constexpr uint8_t SLAVE_ADDRESS = 1;
static constexpr uint8_t ERROR_MASK = 0x80;
static constexpr size_t MAX_REGISTERS_PER_REQUEST = 100;
static constexpr size_t MIN_FRAME_SIZE = 4;
static constexpr size_t MAX_FRAME_SIZE = 512;

// WaterFurnace protocol breakpoints — queries cannot span these boundaries
static constexpr uint16_t BREAKPOINT_1 = 12100;
static constexpr uint16_t BREAKPOINT_2 = 12500;

// --- Structured response ---
// rw-wf returns raw uint16_t vectors and leaves the caller to correlate addresses.
// We return address-value pairs so the caller never has to track expected addresses.

struct RegisterValue {
  uint16_t address;
  uint16_t value;
};

struct ParsedResponse {
  uint8_t function_code;
  bool is_error;
  uint8_t error_code;              // Only valid when is_error == true
  std::vector<RegisterValue> registers;  // Only valid for read responses
};

// --- CRC ---
uint16_t crc16(const uint8_t *data, size_t len);

// --- Frame building (all return complete RTU frames with CRC) ---

/// Build func 0x41 request: read contiguous register ranges.
/// Each pair is (start_address, count). Caller must ensure all ranges
/// are within the same breakpoint segment.
std::vector<uint8_t> build_read_ranges_request(
    const std::vector<std::pair<uint16_t, uint16_t>> &ranges);

/// Build func 0x42 request: read individual discontiguous registers.
/// Returns empty vector if addresses.size() > MAX_REGISTERS_PER_REQUEST.
std::vector<uint8_t> build_read_registers_request(
    const std::vector<uint16_t> &addresses);

/// Build func 0x06 request: write single register.
std::vector<uint8_t> build_write_single_request(uint16_t address, uint16_t value);

/// Build func 0x43 request: write multiple discontiguous registers.
/// Each pair is (address, value). Returns empty vector if writes is empty.
std::vector<uint8_t> build_write_multi_request(
    const std::vector<std::pair<uint16_t, uint16_t>> &writes);

// --- Frame validation ---
bool validate_frame_crc(const uint8_t *data, size_t len);
bool is_error_response(uint8_t function_code);

/// Determine the expected total frame size from a partial receive buffer.
/// Returns 0 if more bytes are needed to determine the size.
/// This is something rw-wf duplicates in the hub — we put it in protocol.
size_t expected_frame_size(const uint8_t *data, size_t available);

// --- Response parsing ---

/// Parse raw response bytes into register values.
/// rw-wf's parse_register_values() returns raw uint16 values — the caller
/// must separately track expected_addresses_ and zip them together.
/// Our version takes the expected addresses and returns RegisterValue pairs.
std::vector<RegisterValue> parse_response_with_addresses(
    const uint8_t *payload, size_t payload_len,
    const std::vector<uint16_t> &expected_addresses);

/// Parse a complete frame into a structured response.
/// Validates CRC, extracts function code, handles error responses.
/// For read responses, requires expected_addresses to map values back.
ParsedResponse parse_frame(
    const uint8_t *frame, size_t frame_len,
    const std::vector<uint16_t> &expected_addresses = {});

// --- Breakpoint-aware helpers (rw-wf doesn't have these) ---

/// Determine which breakpoint segment an address falls in.
/// Returns 0 (< 12100), 1 (12100–12499), or 2 (>= 12500).
uint8_t address_segment(uint16_t address);

/// Split a list of addresses into per-segment groups.
/// Returns three vectors: [segment 0 addrs, segment 1 addrs, segment 2 addrs].
/// Each segment's addresses are sorted.
std::array<std::vector<uint16_t>, 3> split_by_segment(
    const std::vector<uint16_t> &addresses);

/// Merge sorted addresses into contiguous ranges where gaps <= max_gap.
/// Useful for converting individual addresses into efficient func 0x41 ranges.
std::vector<std::pair<uint16_t, uint16_t>> merge_to_ranges(
    const std::vector<uint16_t> &sorted_addresses, uint16_t max_gap = 8);

}  // namespace waterfurnace_aurora
}  // namespace esphome
```

#### 1.2 — Create `protocol.cpp` with implementations

Move from `waterfurnace_aurora.cpp`:
- `calculate_crc()` → `crc16()`
- Frame building logic from `read_specific_registers()`, `read_register_ranges()`, `write_holding_register()`
- CRC validation from `wait_for_response()`
- Response parsing — but enhanced to return `ParsedResponse` with address correlation built in
- `expected_frame_size()` — the frame-size-from-function-code logic that rw-wf duplicates in `read_frame_()`
- Breakpoint splitting and range merging — currently in hub's polling setup code

**Key improvement:** The protocol module owns *all* WaterFurnace-specific framing knowledge. The hub becomes a thin transport layer (send bytes, receive bytes, dispatch results).

#### 1.3 — Update hub to use protocol module

Replace inline CRC/frame logic in `send_and_receive()` and the Modbus methods with protocol function calls. The hub:
- Calls `build_read_registers_request()` / `build_read_ranges_request()` to get frames
- Sends via UART with flow control
- Feeds received bytes to `expected_frame_size()` to know when a frame is complete
- Calls `parse_frame()` to get `ParsedResponse` with address-value pairs
- Dispatches `RegisterValue` entries to listeners

No more manually tracking `expected_addresses_` vectors in the hub — the protocol module handles the correlation.

#### 1.4 — Validate no behavior change

Compile, flash, verify all sensors/controls still work identically. The protocol separation is a pure refactor — zero behavioral change.

---

### Phase 2: Register Definitions Module

**Goal:** Extract register knowledge into a standalone `registers.h` — improving on rw-wf by using constexpr metadata tables, compile-time capability resolution, and sentinel-aware conversions.
**Effort:** 3–4 hours | **Risk:** Low | **Impact:** High (data-driven sensors, cleaner hub, fully testable)

#### Where rw-wf falls short

rw-wf's `registers.h` (470 lines) is a solid start but has design weaknesses:

1. **Runtime string matching for types/capabilities.** Sensors store `register_type_` and `capability_` as `std::string` members (heap-allocated on every entity). The sensor's `on_register_value_()` does runtime `if (this->register_type_ == "signed_tenths")` string comparisons *on every poll cycle*. That's 56 sensors × string compares × every 10 seconds.

2. **No sentinel handling in conversions.** `convert_register()` returns the raw converted float. Sentinel detection (-999.9 / 999.9 → NaN) is done separately in each sensor entity's callback. This means every new entity type that handles temperature must remember to add the sentinel check.

3. **Flat constants only.** Register addresses are bare `constexpr` values with no associated metadata. The Python `__init__.py` has a `SENSOR_TYPES` dict mapping each sensor to `(register, type, is_32bit, capability)`, but this metadata doesn't exist in C++ — so the C++ sensor entity needs 4 separate setter calls from codegen (`set_register_address`, `set_register_type`, `set_is_32bit`, `set_capability`).

4. **No read→write address mapping.** The relationship between read addresses (e.g., heating setpoint at reg 745) and write addresses (e.g., reg 12619) is implicit — scattered across climate and number entity code.

Our register module will address all of these.

#### 2.1 — Create `registers.h` with constexpr metadata

Move from `waterfurnace_aurora.h` into `registers.h`:

- All `namespace registers` constants (~80 register addresses)
- System output/input/AXB bitmask constants and `BitLabel` tables
- VS Drive flag constants
- All enums (`HeatingMode`, `FanMode`, `BlowerType`, `PumpType`, `ZoneCall`, `ZonePriority`, `ZoneSize`)
- `IZ2ZoneData` struct
- Fault code table

#### 2.2 — Use enum-based types instead of runtime strings

Replace rw-wf's string-based type system with compile-time enums:

```cpp
// rw-wf: std::string register_type_ = "signed_tenths"; // heap-allocated, strcmp at runtime
// Ours: constexpr enum, zero overhead at runtime

enum class RegisterType : uint8_t {
  UNSIGNED,        // Raw uint16
  SIGNED,          // int16
  TENTHS,          // uint16 / 10.0
  SIGNED_TENTHS,   // int16 / 10.0  (sentinel-aware)
  HUNDREDTHS,      // uint16 / 100.0
  BOOLEAN,         // 0 or 1
  UINT32,          // Two consecutive registers: (hi << 16) | lo
  INT32,           // Two consecutive registers, signed
};
```

#### 2.3 — Sentinel-aware conversions built into the conversion layer

rw-wf detects sentinel values (-999.9 / 999.9) *after* conversion in each sensor entity. We build it into `convert_register()` so no entity ever needs to think about it:

```cpp
/// Convert raw register to float. Returns NAN for sentinel values.
/// rw-wf does sentinel detection per-entity; we do it once here.
inline float convert_register(uint16_t raw, RegisterType type) {
  float result;
  switch (type) {
    case RegisterType::SIGNED_TENTHS:
      result = static_cast<int16_t>(raw) / 10.0f;
      // Sentinel: -999.9 (raw 0xD8F1) or 999.9 (raw 0x270F)
      if (raw == 0xD8F1 || raw == 0x270F) return NAN;
      return result;
    case RegisterType::TENTHS:
      result = raw / 10.0f;
      if (raw == 0x270F) return NAN;  // 999.9 sentinel
      return result;
    case RegisterType::UNSIGNED:
      return static_cast<float>(raw);
    case RegisterType::SIGNED:
      return static_cast<float>(static_cast<int16_t>(raw));
    case RegisterType::HUNDREDTHS:
      return raw / 100.0f;
    case RegisterType::BOOLEAN:
      return (raw != 0) ? 1.0f : 0.0f;
    default:
      return static_cast<float>(raw);
  }
}
```

#### 2.4 — `RegisterCapability` enum with compile-time resolution

Same concept as rw-wf but using enum directly instead of string → enum conversion at setup:

```cpp
enum class RegisterCapability : uint8_t {
  NONE,               // Always pollable
  AWL_THERMOSTAT,     // Thermostat v3.0+
  AWL_AXB,            // AXB v2.0+
  AWL_COMMUNICATING,  // Thermostat v3.0+ OR IZ2 v2.0+
  AXB,                // AXB present
  REFRIGERATION,      // AXB + energy_monitor >= 1
  ENERGY,             // AXB + energy_monitor == 2
  VS_DRIVE,           // VS drive present
  IZ2,                // IZ2 with AWL v2.0+
};
```

In Python codegen, we emit the enum value directly: `cg.add(var.set_capability(RegisterCapability::AXB))` — no runtime string parsing needed. rw-wf passes `"axb"` as a string and calls `capability_from_string()` at startup.

#### 2.5 — Constexpr register metadata table (optional, stretch)

Consider a constexpr table encoding register metadata for data-driven entity creation:

```cpp
struct RegisterInfo {
  uint16_t address;
  RegisterType type;
  RegisterCapability capability;
  uint16_t write_address;    // 0 = read-only
  const char *name;          // For logging/debug only
};

// Example entries:
static constexpr RegisterInfo REGISTER_TABLE[] = {
  {1111, RegisterType::SIGNED_TENTHS, RegisterCapability::AXB, 0, "entering_water_temp"},
  {745,  RegisterType::TENTHS, RegisterCapability::AWL_THERMOSTAT, 12619, "heating_setpoint"},
  // ...
};
```

This encodes read→write address mapping (currently scattered), capability requirements (currently ad-hoc), and data types (currently in Python only) in one place. Makes it trivial to add new sensors — one table row instead of touching 3+ files. **This is a stretch goal** — evaluate effort vs. benefit during implementation.

#### 2.6 — IZ2 zone extraction helpers

Move from hub into registers module as constexpr/inline functions:

```cpp
uint8_t iz2_extract_mode(uint16_t config2);          // 3-bit mask (our improvement over Ruby gem's 2-bit)
uint8_t iz2_extract_fan_mode(uint16_t config1);
uint8_t iz2_extract_cooling_setpoint(uint16_t config1);
uint8_t iz2_extract_heating_setpoint(uint16_t config1, uint16_t config2);
bool iz2_damper_open(uint16_t config2);
```

#### 2.7 — Setup-phase register group definitions

```cpp
std::vector<std::pair<uint16_t, uint16_t>> get_system_id_ranges();
std::vector<std::pair<uint16_t, uint16_t>> get_component_detect_ranges();
```

#### 2.8 — Bitmask-to-string helpers

Move the `BitLabel` tables and `bitmask_to_string()` into registers — they're protocol knowledge, not hub logic:

```cpp
struct BitLabel {
  uint16_t mask;
  const char *label;
};

static constexpr BitLabel OUTPUT_BITS[] = { ... };
static constexpr BitLabel INPUT_BITS[] = { ... };

std::string bitmask_to_string(uint16_t value, const BitLabel *bits, size_t count);
```

---

### Phase 3: Async State Machine

**Goal:** Replace blocking Modbus I/O with a non-blocking state machine.
**Effort:** 8–12 hours | **Risk:** Medium-High | **Impact:** Very High (eliminates all `loop()` blocking)

This is the single most impactful architectural change. It eliminates the worst-case 4.5-second block in `update()` and aligns with ESPHome best practices.

#### 3.1 — Define state machine

```cpp
enum class State : uint8_t {
  SETUP_READ_ID,           // Initial system identification
  SETUP_DETECT_COMPONENTS, // Hardware detection
  IDLE,                    // Waiting for next poll cycle
  WAITING_RESPONSE,        // Request sent, collecting response bytes
  ERROR_BACKOFF,           // Communication error, waiting before retry
};
```

#### 3.2 — Restructure `setup()` to be non-blocking

Current `setup()` performs blocking Modbus reads for system ID and hardware detection. Move this to the state machine:

- `setup()`: Initialize UART, flow control pin, set state to `SETUP_READ_ID`
- `loop()`: Drive state machine — send system ID request, collect response, parse, transition to `SETUP_DETECT_COMPONENTS`, etc.

#### 3.3 — Restructure `update()` to be non-blocking

Current: `update()` → `refresh_all_data()` → blocking `send_and_receive()` calls.

New:
- `update()`: If state is `IDLE`, start a new poll cycle (set `current_poll_group_ = 0`, send first group request)
- `loop()`: In `WAITING_RESPONSE` state, read available bytes into `rx_buffer_`. When a complete frame is detected, parse it, dispatch register values, advance to next poll group or back to `IDLE`.

#### 3.4 — Add non-blocking frame reader

```cpp
bool read_frame_(std::vector<uint8_t> &frame);  // Returns true when complete frame available
```

Incrementally reads bytes from UART into `rx_buffer_` in `loop()`. Determines expected frame size from function code + byte count. Returns `true` only when the full frame (including CRC) is buffered.

#### 3.5 — Add response timeout handling

If no complete response within `RESPONSE_TIMEOUT` (2 seconds):
- Log warning
- Erase expected addresses from register cache (staleness protection)
- Transition to `ERROR_BACKOFF` state
- After `ERROR_BACKOFF_TIME` (5 seconds), retry or resume

#### 3.6 — Add deferred setup callbacks for child entities

Child entities (climate, switch, number) need hardware detection results before registering listeners. Add:

```cpp
void register_setup_callback(std::function<void()> callback);
```

Called when `setup_complete_` becomes true. This ensures child entities can wait for IZ2/AXB/VS detection before registering their listeners.

#### 3.7 — Add write queue with non-blocking dispatch

Replace direct `write_holding_register()` with:

```cpp
void write_register(uint16_t addr, uint16_t value);  // Queues the write
void process_pending_writes_();  // Called from loop() when IDLE
```

Writes are queued in `pending_writes_` vector and sent when the bus is idle. Consider using func 67 (batch write) when multiple writes are pending.

#### 3.8 — Add write cooldown to climate entity

Prevent stale read-backs from reverting optimistic UI updates:

```cpp
static constexpr uint32_t WRITE_COOLDOWN_MS = 10000;
uint32_t last_mode_write_{0};
uint32_t last_heating_sp_write_{0};
uint32_t last_cooling_sp_write_{0};
uint32_t last_fan_write_{0};

bool in_cooldown_(uint32_t last_write) const {
  return last_write != 0 && (millis() - last_write) < WRITE_COOLDOWN_MS;
}
```

In register callbacks, check cooldown before updating state from read-back values.

#### 3.9 — Preserve three-tier polling within state machine

Keep our fast/medium/slow polling tiers but implement them within the state machine:

- Each poll cycle sends one tier's worth of registers
- `update()` (called every 5s) triggers fast-tier poll
- Medium and slow tiers use internal counters (every 6th and 60th cycle)
- Only registers with active listeners are included in each tier

#### 3.10 — Add connected binary sensor

```yaml
waterfurnace_aurora:
  id: aurora
  connected:
    name: "Heat Pump Connected"
  connected_timeout: 30s
```

Track `last_successful_response_` timestamp. Publish `false` when no successful response within timeout. Publish `true` on any successful response.

---

### Phase 4: Testing Infrastructure

**Goal:** Add unit tests and CI/CD pipeline using Catch2 v3.
**Effort:** 6–8 hours | **Risk:** Low | **Impact:** High (catches regressions, validates protocol correctness)

#### Why Catch2 over gtest

rw-wf uses Google Test, which requires `apt-get install libgtest-dev` or building from source. We use **Catch2 v3** because:

- **Vendorable** — single `catch_amalgamated.hpp` + `catch_amalgamated.cpp` checked into the repo. Zero external dependencies. Contributors clone and `make test` — nothing to install.
- **Cleaner syntax** — `REQUIRE(crc16(data, 6) == 0x0A84)` reads better than `EXPECT_EQ(crc16(data, 6), 0x0A84)`
- **SECTION nesting** — perfect for register conversion tests with shared setup:
  ```cpp
  TEST_CASE("convert_register") {
    SECTION("signed_tenths") {
      REQUIRE(convert_register(700, RegisterType::SIGNED_TENTHS) == Approx(70.0f));
      SECTION("sentinel -999.9 becomes NAN") {
        REQUIRE(std::isnan(convert_register(0xD8F1, RegisterType::SIGNED_TENTHS)));
      }
    }
  }
  ```
- **Better failure messages** — shows expression, LHS, RHS automatically
- **Built-in benchmarking** — `BENCHMARK("CRC16 90 registers") { return crc16(data, 184); }` if we ever profile

#### 4.1 — Vendor Catch2 v3

Download the amalgamated build into `tests/`:
```
tests/
  catch_amalgamated.hpp    # Single header (~600KB)
  catch_amalgamated.cpp    # Single source (~200KB)
```

No submodules, no package manager, no system install. Works on Linux, macOS, and Windows CI.

#### 4.2 — Create minimal ESPHome mock headers

Since our `protocol.h` and `registers.h` have **zero ESPHome dependencies** (by design from Phases 1 & 2), we only need mocks for hub-level tests:

```
tests/mocks/
  esphome/core/component.h   # Component, PollingComponent stubs
  esphome/core/log.h          # ESP_LOG macros → no-op or printf
  esphome/core/hal.h          # millis() stub
  esphome_types.h             # Common typedefs (optional, GPIOPin etc.)
```

Protocol and register tests compile with **no mocks at all** — just `#include "protocol.h"` and `#include "registers.h"` directly. This is a direct benefit of the Phase 1/2 separation.

#### 4.3 — Create protocol tests (`tests/test_protocol.cpp`)

```cpp
#include "catch_amalgamated.hpp"
#include "protocol.h"

using namespace esphome::waterfurnace_aurora;

TEST_CASE("CRC16") {
  SECTION("known Modbus vector") {
    uint8_t data[] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x01};
    REQUIRE(crc16(data, sizeof(data)) == 0x0A84);
  }
  SECTION("func 65 request") { /* ... */ }
  SECTION("empty input") {
    REQUIRE(crc16(nullptr, 0) == 0xFFFF);  // Edge case rw-wf doesn't test
  }
}

TEST_CASE("Frame building") {
  SECTION("read_ranges validates and builds correctly") { /* ... */ }
  SECTION("read_registers rejects >100 addresses") {
    // Our protocol module validates this; rw-wf's doesn't
    auto frame = build_read_registers_request(std::vector<uint16_t>(101, 0));
    REQUIRE(frame.empty());
  }
  SECTION("write_multi batches correctly") { /* ... */ }
}

TEST_CASE("Response parsing with address correlation") {
  // rw-wf tests raw value parsing only.
  // We test the full parse_frame() → ParsedResponse → RegisterValue pipeline.
  SECTION("func 66 response maps values to addresses") { /* ... */ }
  SECTION("error response has is_error flag and code") { /* ... */ }
  SECTION("CRC mismatch returns error") { /* ... */ }
}

TEST_CASE("expected_frame_size") {
  // Tests the non-blocking frame completeness check
  SECTION("func 65/66: needs byte_count to determine size") { /* ... */ }
  SECTION("func 6 echo: always 8 bytes") { /* ... */ }
  SECTION("error response: always 5 bytes") { /* ... */ }
}

TEST_CASE("Breakpoint-aware helpers") {
  SECTION("address_segment classifies correctly") {
    REQUIRE(address_segment(742) == 0);
    REQUIRE(address_segment(12309) == 1);
    REQUIRE(address_segment(31003) == 2);
  }
  SECTION("split_by_segment groups addresses") { /* ... */ }
  SECTION("merge_to_ranges with gap optimization") {
    auto ranges = merge_to_ranges({19, 20, 30, 31}, 8);
    REQUIRE(ranges.size() == 1);  // Gap of 10 between 20 and 30 → single range
  }
}
```

#### 4.4 — Create register tests (`tests/test_registers.cpp`)

```cpp
#include "catch_amalgamated.hpp"
#include "registers.h"

using namespace esphome::waterfurnace_aurora;

TEST_CASE("Register type conversions") {
  SECTION("unsigned") {
    REQUIRE(convert_register(240, RegisterType::UNSIGNED) == 240.0f);
  }
  SECTION("signed_tenths") {
    REQUIRE(convert_register(700, RegisterType::SIGNED_TENTHS) == Approx(70.0f));
    SECTION("negative values") {
      uint16_t neg = static_cast<uint16_t>(static_cast<int16_t>(-105));
      REQUIRE(convert_register(neg, RegisterType::SIGNED_TENTHS) == Approx(-10.5f));
    }
    SECTION("sentinel -999.9 → NAN") {
      REQUIRE(std::isnan(convert_register(0xD8F1, RegisterType::SIGNED_TENTHS)));
    }
    SECTION("sentinel 999.9 → NAN") {
      REQUIRE(std::isnan(convert_register(0x270F, RegisterType::SIGNED_TENTHS)));
    }
  }
  SECTION("boolean") {
    REQUIRE(convert_register(0, RegisterType::BOOLEAN) == 0.0f);
    REQUIRE(convert_register(42, RegisterType::BOOLEAN) == 1.0f);
  }
}

TEST_CASE("32-bit register assembly") {
  SECTION("uint32") { /* ... */ }
  SECTION("int32 negative") {
    REQUIRE(to_int32(0xFFFF, 0xFFFF) == -1);
  }
}

TEST_CASE("IZ2 zone extraction") {
  SECTION("mode uses 3-bit mask for E-Heat support") {
    REQUIRE(iz2_extract_mode(0x0400) == 4);  // MODE_EHEAT
    // rw-wf also tests this, but we verify the *fix* over the Ruby gem's 2-bit mask
    REQUIRE(iz2_extract_mode(0x2410) == 4);   // Real E-Heat config2 from device
  }
  SECTION("fan mode") { /* ... */ }
  SECTION("setpoints") { /* ... */ }
  SECTION("damper") { /* ... */ }
}

TEST_CASE("Fault codes") {
  REQUIRE(std::string(get_fault_description(1)) == "Input Error");
  REQUIRE(std::string(get_fault_description(99)) == "System Reset");
  REQUIRE(std::string(get_fault_description(50)).find("Unknown") != std::string::npos);
}

TEST_CASE("Setup register groups") {
  SECTION("system ID range total count") {
    auto ranges = get_system_id_ranges();
    size_t total = 0;
    for (const auto &r : ranges) total += r.second;
    REQUIRE(total > 20);  // Sanity check
  }
}
```

#### 4.5 — Create Makefile

```makefile
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra \
           -I../../components/waterfurnace_aurora \
           -I.
SRCS_CATCH = catch_amalgamated.cpp

.PHONY: test clean

test: test_protocol test_registers
	./test_protocol && ./test_registers

test_protocol: test_protocol.cpp ../../components/waterfurnace_aurora/protocol.cpp $(SRCS_CATCH)
	$(CXX) $(CXXFLAGS) -o $@ $^

test_registers: test_registers.cpp $(SRCS_CATCH)
	$(CXX) $(CXXFLAGS) -o $@ $^

clean:
	rm -f test_protocol test_registers
```

No `-lgtest`, no `-pthread`, no system package installs. Clone + `make test`.

#### 4.6 — Add Docker-based integration tests (stretch goal)

Using rw-wf's approach: run the Ruby gem's ModBus TCP server as a mock, send real requests from our C++ code via TCP socket wrapper, verify responses match expected fixture data. This validates end-to-end protocol correctness against the reference implementation.

#### 4.7 — Add CI workflow (`.github/workflows/ci.yml`)

```yaml
on: [pull_request, push]
jobs:
  unit-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - run: cd tests && make test
      # No apt-get install, no pip install — Catch2 is vendored

  esphome-compile:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        esphome: [stable, beta, dev]
    steps:
      - uses: actions/checkout@v4
      - run: pip install esphome
      - run: esphome compile tests/waterfurnace-test.yaml
```

---

### Phase 5: Developer & User Experience

**Goal:** Improve installation, monitoring, and debugging experience.
**Effort:** 4–6 hours | **Risk:** Low | **Impact:** Medium

#### 5.1 — Add HA API `write_register` service

Expose a custom service for direct register reads/writes from Home Assistant (useful for debugging and advanced users):

```cpp
#ifdef USE_API_CUSTOM_SERVICES
register_service(&WaterFurnaceAurora::on_write_register_service_, "write_register",
                 {"address", "value"});
#endif
```

Requires `api: custom_services: true` in YAML. Validates address/value ranges.

#### 5.2 — Add web server sorting groups

Organize the web UI with logical groupings:

```yaml
web_server:
  version: 3
  local: true
  sorting_groups:
    - id: group_thermostat
      name: "Thermostat"
      sorting_weight: 10
    - id: group_performance
      name: "Performance"
      sorting_weight: 20
    - id: group_power
      name: "Power & Electrical"
      sorting_weight: 30
    - id: group_vs_drive
      name: "VS Drive"
      sorting_weight: 40
    - id: group_controls
      name: "Controls"
      sorting_weight: 50
    - id: group_diagnostics
      name: "Diagnostics"
      sorting_weight: 60
```

#### 5.3 — Add one-click web install page

Create a `static/` directory with ESP Web Tools integration for browser-based firmware flashing:

```html
<esp-web-install-button manifest="firmware/manifest.json">
</esp-web-install-button>
```

Requires building factory firmware with `esphome compile --format=factory` and hosting on GitHub Pages.

#### 5.4 — Add release tooling

Create `release.sh` for automated releases:
- Validates semantic versioning
- Bumps version in YAML config
- Creates git tag
- Pushes to remote
- Creates GitHub Release via `gh`
- Pre-release detection (versions containing `-`)

#### 5.5 — Add `DEVELOPMENT.md`

Document local development workflow:
- How to run unit tests
- How to compile with local component source
- How to test against a branch
- How to use `refresh: 0s` for development
- Docker integration test instructions

---

### Phase 6: Feature Parity Additions

**Goal:** Add remaining sensors and controls not yet implemented.
**Effort:** Incremental, ~30min each | **Risk:** Low | **Impact:** Low-Medium per item

These are independent additions that can be done at any time. Prioritize based on user requests.

#### Sensors (read-only, safe to add anytime)

| # | Feature | Register(s) | Capability | Notes |
|---|---------|-------------|------------|-------|
| 6.1 | VS compressor speed desired | 3000 | vs_drive | Useful for monitoring ramp behavior vs actual (3001) |
| 6.2 | IZ2 desired compressor speed | 564 | vs_drive + iz2 | Read when VSDrive + IZ2 |
| 6.3 | IZ2 desired blower speed | 565 | iz2 | Converts: 1=25%, 2=40%, 3=55%, 4=70%, 5=85%, 6=100% |
| 6.4 | VS pump output | 325 | axb + vs_pump | Current pump speed %, only valid when AWL AXB |
| 6.5 | Aux heat stage (0/1/2) | Derived from reg 30 | none | Computed from EH1+EH2 bits |
| 6.6 | Last lockout fault | 26 | none | High bit = locked out; bits 0-14 = fault code |
| 6.7 | Outputs at lockout | 27 | none | Bitmask text sensor (already in rw-wf) |
| 6.8 | Inputs at lockout | 28 | none | Bitmask text sensor (already in rw-wf) |

#### Controls (write, need testing)

| # | Feature | Register | Range | Notes |
|---|---------|----------|-------|-------|
| 6.9 | Line voltage setting | 112 (write) | 90-635V | Read-only currently |
| 6.10 | Cooling airflow adjustment | 346 (write, signed) | — | Not implemented |
| 6.11 | Loop pressure trip | 419 (write, tenths) | — | Not implemented |
| 6.12 | Manual operation | 3002 (write, bitmask) | — | Dangerous — gate behind `enable_manual_control: true` |

---

## 4. What NOT to Change

Things that are working well and should be preserved:

| Feature | Reason to Keep |
|---------|----------------|
| **Three-tier adaptive polling** | Better data freshness semantics than flat polling. rw-wf polls everything at the same rate. |
| **Flat sorted vector register store** | More memory-efficient than `std::map`. rw-wf uses `std::map`. |
| **DRY publish helpers** | `publish_sensor()`, `publish_sensor_tenths()`, etc. eliminate 50+ repetitive patterns |
| **Shared `aurora_climate_utils.h`** | Clean separation of mode mapping, used by both climate classes |
| **Generic `AuroraNumber` class** | Single class handles 11 number controls via type dispatch — avoids 11 separate classes |
| **Hardware override YAML options** | `has_axb: true`, `has_vs_drive: true` etc. — lets users force detection when auto-detect fails |
| **Broader hardware support** | Don't lock to a specific board. Keep MAX485 flow control pin support. |
| **Derived sensors (COP, delta-T)** | Unique value-add that rw-wf doesn't have |

---

## Phase Summary

| Phase | Effort | Risk | Impact | Depends On |
|-------|--------|------|--------|------------|
| 1: Protocol Separation | 3–4 hr | Low | High — enables testing, cleaner code, smarter protocol layer than rw-wf | Nothing |
| 2: Register Definitions | 3–4 hr | Low | High — compile-time types, sentinel-aware conversions, data-driven potential | Nothing (can parallel with Phase 1) |
| 3: Async State Machine | 8–12 hr | Medium-High | Very High — eliminates all blocking | Phases 1 & 2 (cleaner to build on separated modules) |
| 4: Testing (Catch2) | 6–8 hr | Low | High — zero-dependency vendored tests, catches regressions | Phases 1 & 2 (tests target separated modules) |
| 5: Developer Experience | 4–6 hr | Low | Medium — better UX | Phase 3 (connected sensor needs state machine) |
| 6: Feature Additions | ~30min each | Low | Low-Medium per item | Nothing (independent) |

### Recommended Approach

1. **Do Phases 1 & 2 together** — These are pure refactors (extract, don't change behavior). They reduce `waterfurnace_aurora.h` from 852 to ~400 lines and `waterfurnace_aurora.cpp` from 2034 to ~1400 lines, making the state machine rewrite (Phase 3) much more manageable.

2. **Do Phase 4 immediately after Phases 1 & 2** — Write tests for the newly separated `protocol.h/cpp` and `registers.h` before changing any behavior. This gives you a safety net for Phase 3.

3. **Do Phase 3 (state machine)** — The biggest change. With protocol/registers already separated and tested, this focuses purely on the hub's communication orchestration. The separated protocol module's frame-building and parsing functions are reused directly.

4. **Do Phase 5 incrementally** — Connected sensor is the highest-value item (do it as part of Phase 3). Web install, release tooling, and API services can be added anytime.

5. **Do Phase 6 on demand** — Add sensors/controls as users request them. Each is independent and ~30 minutes of work.
