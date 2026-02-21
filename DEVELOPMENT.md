# Development Guide

## Prerequisites

- **C++ compiler** with C++17 support (g++ or clang++)
- **Python 3.10+** with pip
- **ESPHome** (`pip install esphome`)

## Repository Structure

```
components/waterfurnace_aurora/
├── protocol.h / protocol.cpp      # Modbus protocol (zero ESPHome deps)
├── registers.h / registers.cpp    # Register definitions (zero ESPHome deps)
├── waterfurnace_aurora.h / .cpp   # Hub: async state machine, sensor publishing
├── climate/                       # Main thermostat + IZ2 zone thermostats
├── number/                        # 11 number controls (DHW, blower, pump, etc.)
├── switch/                        # DHW enable switch
├── button/                        # Clear fault history button
├── sensor/__init__.py             # 60 sensor entities
├── binary_sensor/__init__.py      # 12 binary sensor entities
└── text_sensor/__init__.py        # 17 text sensor entities

tests/
├── Makefile                       # Build + run all tests
├── catch_amalgamated.hpp / .cpp   # Catch2 v3.7.1 (vendored)
├── test_protocol.cpp              # Protocol tests (13 test cases)
├── test_registers.cpp             # Register tests (27 test cases)
├── test_hub.cpp                   # Hub state machine tests (7 test cases)
├── mock_impl.cpp                  # Mock millis() storage
├── mocks/esphome/                 # Mock ESPHome headers for hub tests
└── waterfurnace-test.yaml         # CI compilation test config
```

## Running Unit Tests

No system packages needed — Catch2 is vendored.

```bash
cd tests
make test
```

This builds and runs three test binaries:
- `test_protocol` — CRC, frame building, response parsing, breakpoint helpers
- `test_registers` — Type conversions, IZ2 extraction, fault codes, bitmasks
- `test_hub` — State machine, write queue validation, setup flow

## Architecture

### Three-Layer Design

1. **Protocol layer** (`protocol.h/cpp`) — Pure Modbus RTU framing. No ESPHome dependencies. Handles CRC16, frame building for func 0x03/0x06/0x41/0x42/0x43, response parsing with address correlation, and breakpoint-aware request splitting.

2. **Register layer** (`registers.h/cpp`) — WaterFurnace domain knowledge. No ESPHome dependencies. Contains register addresses, enums, bitmask tables, fault codes, IZ2 zone extraction, and sentinel-aware conversions.

3. **Hub layer** (`waterfurnace_aurora.h/cpp`) — ESPHome integration. Async state machine (`SETUP_READ_ID → SETUP_DETECT_COMPONENTS → SETUP_DETECT_VS → IDLE ↔ WAITING_RESPONSE ↔ ERROR_BACKOFF`), three-tier adaptive polling, write queue with cooldowns, and sensor publication.

### Key Design Decisions

- **Zero blocking I/O** — The state machine never blocks. Bytes are read incrementally in `loop()`. No `delay()` calls in the communication path.
- **Write cooldowns** — 10-second cooldown after writes prevents stale read-backs from reverting optimistic UI updates.
- **Connected sensor** — Tracks RS-485 communication health with configurable timeout (default 30s).
- **Flat sorted vector** — `RegisterMap` uses binary search on a sorted `vector<pair>` instead of `std::map`, saving ~4KB heap fragmentation on ESP32.

## Compiling with ESPHome

### Local Development

Use a YAML config with `external_components` pointing to your local checkout:

```yaml
external_components:
  - source:
      type: local
      path: /path/to/esphome_waterfurnace_aurora/components
```

### Testing Against a Branch

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/daemonp/esphome_waterfurnace_aurora
      ref: your-branch-name
    refresh: 0s  # Always pull latest during development
```

### CI Compilation Check

The test YAML exercises all entity platforms:

```bash
esphome compile tests/waterfurnace-test.yaml
```

## HA API Custom Service

When `api: custom_services: true` is set in YAML, the component registers a `write_register` service in Home Assistant:

```yaml
# Call from HA Developer Tools > Services:
service: esphome.<node_name>_write_register
data:
  address: 12619    # Register address (0-65535)
  value: 700        # Register value (0-65535)
```

This is useful for debugging and advanced users who need direct register access.

## Adding a New Sensor

1. **Add register address** to `registers.h` (if not already defined)
2. **Add sensor pointer** to the hub class in `waterfurnace_aurora.h` (with setter)
3. **Add publish call** in `publish_all_sensors_()` in `waterfurnace_aurora.cpp`
4. **Add poll address** in `build_poll_addresses_()` (choose correct tier)
5. **Add Python config** in the appropriate `__init__.py` (sensor/binary_sensor/text_sensor)
6. **Add to test YAML** in `tests/waterfurnace-test.yaml`
7. **Run tests** — `cd tests && make test`
