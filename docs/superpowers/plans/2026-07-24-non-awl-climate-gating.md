# Non-AWL / Dry-Contact Climate Gating Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use subagent-driven-development (recommended) or executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop publishing garbage thermostat/humidistat targets and reject misleading writes on dry-contact (non-AWL) installs, while keeping climate action + current temperature useful via entering-air fallback.

**Architecture:** Hub-wide capability model matching upstream gem v1.6.4 (`awl_thermostat` = installed + version ≥ 3.0; `has_humidistat_targets` = AWL communicating + humidifier/dehumidifier/VS). Poll, publish, and write paths all gate on those helpers. Climate non-IZ2 path becomes status-oriented when AWL thermostat controls are absent.

**Tech Stack:** C++20 ESPHome component, Catch2 unit tests (`tests/test_hub.cpp`), mock UART harness, docs in `docs/ENTITIES.md` + `README.md`.

**Spec:** `docs/superpowers/specs/2026-07-24-non-awl-climate-gating-design.md`

**Worktree:** `.worktrees/fix-non-awl-climate-gating` on branch `fix/non-awl-climate-gating`

---

## File map

| File | Responsibility |
| ---- | -------------- |
| `components/waterfurnace_aurora/waterfurnace_aurora.h` | `has_thermostat_`, fixed `awl_thermostat()`, public helpers, EAT cache + climate temp helper |
| `components/waterfurnace_aurora/waterfurnace_aurora.cpp` | Detection, poll gates, publish sanitizers, write gates, capability logs |
| `components/waterfurnace_aurora/climate/aurora_climate.cpp` | Non-IZ2 status-oriented update; EAT fallback via hub helper |
| `components/waterfurnace_aurora/climate/aurora_climate_utils.h` | Gate humidity target reads on `has_humidistat_targets()` + sanitize `0x00`/`0xFF` |
| `tests/test_hub.cpp` | Fixture fix (`THERMOSTAT_INSTALLED`), new non-AWL / AWL / humidistat / EAT cases |
| `docs/ENTITIES.md`, `README.md` | Dry-contact user note |

No protocol or register address changes.

---

### Task 1: Fixture fix + capability helpers (TDD)

**Files:**
- Modify: `tests/test_hub.cpp`
- Modify: `components/waterfurnace_aurora/waterfurnace_aurora.h`
- Modify: `components/waterfurnace_aurora/waterfurnace_aurora.cpp` (detection + logs only in this task)

**Critical fixture note:** `drive_setup` defaults every detect register to `3` (`COMPONENT_NOT_INSTALLED`), then sets `THERMOSTAT_VERSION=300`. Today `awl_thermostat()` is version-only so outdoor-temp tests pass. After the gem-correct fix, version 3.00 + installed=3 must be **false**. Update AWL fixtures so installed ≠ 3.

- [ ] **Step 1: Write failing capability tests**

Add near the end of `tests/test_hub.cpp` (before any final blank lines), and extend `TestableHub` if needed:

```cpp
// In TestableHub (top of file), add:
bool test_awl_thermostat() const { return this->awl_thermostat(); }
bool test_has_humidistat_targets() const { return this->has_humidistat_targets(); }
bool test_has_thermostat() const { return this->has_thermostat(); }
void inject_register(uint16_t addr, uint16_t value) {
  reg_set(this->register_cache_, addr, value);
}
void force_publish(const RegisterMap &regs) { this->publish_all_sensors_(); /* see note */ }
```

Prefer calling **public** APIs once they exist (`has_thermostat()`, `has_awl_thermostat_controls()`, `has_humidistat_targets()`, `awl_communicating()`). Use `TestableHub` only for protected pieces still needed (e.g. driving publish via a public test hook if one is added).

Add helper to customize thermostat detect fields:

```cpp
// After drive_setup — variant that sets thermostat installed + version explicitly.
// installed_val: COMPONENT_NOT_INSTALLED (3) = absent; any other status = present (gem).
// version_raw: register 801 units (300 = v3.00).
static void drive_setup_thermostat(WaterFurnaceAurora &hub, uint16_t installed_val,
                                   uint16_t version_raw,
                                   bool humidifier = false) {
  hub.setup();
  hub.loop();
  hub.mock_get_transmitted();
  complete_tx(hub, 0);
  std::vector<uint16_t> id_values(18, 0x2020);
  hub.mock_receive(make_response_03(id_values));
  set_millis(50);
  hub.loop();

  hub.loop();
  auto tx = hub.mock_get_transmitted();
  size_t num_detect_regs = (tx.size() - 4) / 2;
  std::vector<std::pair<uint16_t, uint16_t>> detect_vals;
  for (size_t i = 0; i < num_detect_regs; i++) {
    uint16_t addr = (tx[2 + i * 2] << 8) | tx[3 + i * 2];
    uint16_t val = 3;
    if (addr == registers::THERMOSTAT_INSTALLED) val = installed_val;
    if (addr == registers::THERMOSTAT_VERSION) val = version_raw;
    if (addr == registers::BLOWER_TYPE) val = 0;
    if (addr == registers::ENERGY_MONITOR) val = 0;
    if (addr == registers::PUMP_TYPE) val = 0;
    // DIP 33: accessory_relay humidifier = bit pattern from parse_dip_switches.
    // Use a known raw value only if humidifier requested — see registers tests /
    // parse_dip_switches. If unsure, set has_humidifier via a small TestableHub
    // setter added for tests, or craft DIP raw that yields AccessoryRelay::HUMIDIFIER.
    if (addr == registers::DIP_SWITCH_STATUS && humidifier) {
      // accessory_relay field == HUMIDIFIER (value 1 in low bits per registers.h enum).
      // Match an existing DIP fixture if one exists; otherwise set raw so
      // parse_dip_switches().accessory_relay == AccessoryRelay::HUMIDIFIER.
      val = /* known humidifier DIP raw — verify against parse_dip_switches unit tests */;
    }
    if (addr >= 88 && addr <= 91) val = 0x2020;
    detect_vals.emplace_back(addr, val);
  }
  complete_tx(hub, 50);
  hub.mock_receive(make_response_42(detect_vals));
  set_millis(100);
  hub.loop();

  hub.loop();
  hub.loop();
  hub.mock_get_transmitted();
  complete_tx(hub, 100);
  hub.mock_receive(make_response_42({{3001, 0}, {3322, 0}, {3325, 0}}));
  set_millis(150);
  hub.loop();
}
```

**Simpler humidifier path for tests:** add a test-only public override on the hub header under existing override pattern, or set via `TestableHub`:

```cpp
// waterfurnace_aurora.h near other overrides — only if DIP crafting is painful:
// void set_has_humidifier_for_test(bool v) { this->has_humidifier_ = v; }
// Prefer real detection in drive_setup_thermostat when DIP raw is known.
```

Check `tests/test_registers.cpp` / `parse_dip_switches` for a humidifier DIP raw value and use it.

```cpp
TEST_CASE("awl_thermostat requires installed and version >= 3.0", "[hub][awl][capability]") {
  SECTION("version 3.00 but not installed") {
    TestableHub hub;
    set_millis(0);
    drive_setup_thermostat(hub, registers::COMPONENT_NOT_INSTALLED, 300);
    REQUIRE(hub.is_setup_complete());
    REQUIRE_FALSE(hub.has_thermostat());
    REQUIRE_FALSE(hub.has_awl_thermostat_controls());
    REQUIRE_FALSE(hub.awl_communicating());
  }
  SECTION("installed + version 3.00") {
    TestableHub hub;
    set_millis(0);
    drive_setup_thermostat(hub, 1 /* installed/ok */, 300);
    REQUIRE(hub.has_thermostat());
    REQUIRE(hub.has_awl_thermostat_controls());
    REQUIRE(hub.awl_communicating());
  }
  SECTION("installed + version 0.00 (dry-contact garbage version)") {
    TestableHub hub;
    set_millis(0);
    drive_setup_thermostat(hub, 1, 0);
    REQUIRE(hub.has_thermostat());  // installed bit true
    REQUIRE_FALSE(hub.has_awl_thermostat_controls());
  }
  SECTION("not installed + version 0") {
    TestableHub hub;
    set_millis(0);
    drive_setup_thermostat(hub, registers::COMPONENT_NOT_INSTALLED, 0);
    REQUIRE_FALSE(hub.has_awl_thermostat_controls());
  }
}

TEST_CASE("has_humidistat_targets matches gem", "[hub][awl][humidistat]") {
  SECTION("AWL thermostat without humidifier/dehumidifier/VS") {
    TestableHub hub;
    set_millis(0);
    drive_setup_thermostat(hub, 1, 300, /*humidifier=*/false);
    REQUIRE(hub.has_awl_thermostat_controls());
    REQUIRE_FALSE(hub.has_humidistat_targets());
  }
  SECTION("AWL thermostat with humidifier") {
    TestableHub hub;
    set_millis(0);
    drive_setup_thermostat(hub, 1, 300, /*humidifier=*/true);
    // If DIP path flaky, after setup: hub.set_has_humidifier_for_test(true);
    REQUIRE(hub.has_humidistat_targets());
  }
  SECTION("non-AWL with humidifier") {
    TestableHub hub;
    set_millis(0);
    drive_setup_thermostat(hub, registers::COMPONENT_NOT_INSTALLED, 0, true);
    REQUIRE_FALSE(hub.has_awl_thermostat_controls());
    REQUIRE_FALSE(hub.has_humidistat_targets());
  }
}
```

Also **fix `drive_setup`** so default AWL-style setup marks thermostat installed (preserves existing outdoor-temp and other AWL assumptions):

```cpp
// Inside drive_setup detect_vals loop, add:
if (addr == registers::THERMOSTAT_INSTALLED) val = 1;  // present
if (addr == registers::THERMOSTAT_VERSION) val = 300;
```

Apply the same `THERMOSTAT_INSTALLED = 1` fix to every custom detect loop in `test_hub.cpp` that sets `THERMOSTAT_VERSION = 300` and intends AWL behavior (full setup flow test, outdoor temp test comments, `drive_setup_with_pump`, etc.). Grep for `THERMOSTAT_VERSION` in the file and update each.

- [ ] **Step 2: Run tests — expect compile failure / link failure for missing APIs**

```bash
cd tests && make test_hub 2>&1 | tail -40
```

Expected: missing `has_thermostat`, `has_awl_thermostat_controls`, `has_humidistat_targets`, and/or `COMPONENT_NOT_INSTALLED` access (use `registers::COMPONENT_NOT_INSTALLED` — already in `registers.h`).

- [ ] **Step 3: Implement capability model in header**

In `waterfurnace_aurora.h`:

1. Add public accessors near `awl_communicating()`:

```cpp
bool has_thermostat() const { return this->has_thermostat_; }
bool has_awl_thermostat_controls() const { return this->awl_thermostat(); }
/// Gem: awl_communicating && (humidifier || dehumidifier || VS drive)
bool has_humidistat_targets() const {
  return this->awl_communicating() &&
         (this->has_humidifier_ || this->has_dehumidifier_ || this->has_vs_drive_);
}
/// Ambient if AWL thermostat controls else entering-air cache (may be NAN).
float get_climate_current_temperature_f() const;
float get_entering_air_temperature() const { return this->entering_air_temp_; }
```

2. Fix protected helper:

```cpp
bool awl_thermostat() const {
  return this->has_thermostat_ && this->thermostat_version_ >= 3.0f;
}
```

3. Add members near other hardware flags:

```cpp
bool has_thermostat_{false};
float entering_air_temp_{NAN};  // cache for climate fallback; updated in publish
```

Optional test helper only if DIP crafting fails:

```cpp
#ifdef UNIT_TEST  // only if test makefile already defines something similar — otherwise skip
```

Do **not** invent `UNIT_TEST` if the project doesn't use it; use real detection or a minimal public test setter only as last resort.

- [ ] **Step 4: Store `has_thermostat_` during detect**

In `process_setup_detect_response_` after IZ2 detection block (~line 646), before AWL versions:

```cpp
// Thermostat installed — match gem thermostat? => reg 800 != 3
{
  const uint16_t *val = reg_find(result, registers::THERMOSTAT_INSTALLED);
  if (val) {
    this->has_thermostat_ = (*val != COMPONENT_NOT_INSTALLED);
    ESP_LOGD(TAG, "Thermostat reg %d = %d -> %s", registers::THERMOSTAT_INSTALLED, *val,
             this->has_thermostat_ ? "present" : "absent");
  }
}
```

Implement climate helper in `.cpp`:

```cpp
float WaterFurnaceAurora::get_climate_current_temperature_f() const {
  if (this->awl_thermostat() && !std::isnan(this->ambient_temp_)) {
    return this->ambient_temp_;
  }
  if (!std::isnan(this->entering_air_temp_)) {
    return this->entering_air_temp_;
  }
  return NAN;
}
```

(EAT cache still always NAN until Task 3 publish path — fine for capability tests.)

- [ ] **Step 5: Capability logging in `finish_setup_` and `dump_config`**

`finish_setup_` after existing IZ2 log:

```cpp
ESP_LOGI(TAG, "  Tstat: v%.2f installed=%s awl_thermostat=%s",
         this->thermostat_version_,
         this->has_thermostat_ ? "yes" : "no",
         this->awl_thermostat() ? "yes" : "no");
ESP_LOGI(TAG, "  AWL communicating: %s (thermostat=%s iz2=%s)",
         this->awl_communicating() ? "yes" : "no",
         this->awl_thermostat() ? "yes" : "no",
         this->awl_iz2() ? "yes" : "no");
ESP_LOGI(TAG, "  Humidistat targets: %s (need AWL + humidifier/dehumidifier/VS)",
         this->has_humidistat_targets() ? "yes" : "no");
```

Mirror shorter form in `dump_config()` (include Tstat line — currently missing).

- [ ] **Step 6: Run capability tests**

```bash
cd tests && make test_hub && ./test_hub "[awl]"
```

Expected: PASS for new capability cases; fix any broken outdoor-temp case by ensuring its setup marks thermostat installed.

- [ ] **Step 7: Commit**

```bash
git add components/waterfurnace_aurora/waterfurnace_aurora.h \
        components/waterfurnace_aurora/waterfurnace_aurora.cpp \
        tests/test_hub.cpp \
        docs/superpowers/specs/2026-07-24-non-awl-climate-gating-design.md \
        docs/superpowers/plans/2026-07-24-non-awl-climate-gating.md
git commit -m "$(cat <<'EOF'
feat(hub): gate AWL thermostat on installed + version >= 3.0

Match upstream gem thermostat?/awl_thermostat? and add humidistat-target
capability helpers. Fix test fixtures that assumed version-only AWL.
EOF
)"
```

---

### Task 2: Write gating

**Files:**
- Modify: `components/waterfurnace_aurora/waterfurnace_aurora.cpp` (control methods)
- Modify: `tests/test_hub.cpp`

- [ ] **Step 1: Failing write-gate tests**

```cpp
TEST_CASE("Reject thermostat writes without AWL thermostat", "[hub][awl][write]") {
  TestableHub hub;
  set_millis(0);
  drive_setup_thermostat(hub, registers::COMPONENT_NOT_INSTALLED, 0);
  REQUIRE_FALSE(hub.has_awl_thermostat_controls());

  size_t writes_before = /* pending write count if exposed, else spy via mock TX after loop */;
  REQUIRE_FALSE(hub.set_heating_setpoint(70.0f));
  REQUIRE_FALSE(hub.set_cooling_setpoint(74.0f));
  REQUIRE_FALSE(hub.set_hvac_mode(HeatingMode::HEAT));
  REQUIRE_FALSE(hub.set_fan_mode(FanMode::AUTO));
  REQUIRE_FALSE(hub.set_fan_intermittent_on(5));
  REQUIRE_FALSE(hub.set_fan_intermittent_off(10));
  REQUIRE_FALSE(hub.set_humidification_target(40));
  REQUIRE_FALSE(hub.set_dehumidification_target(50));
  REQUIRE_FALSE(hub.set_humidifier_mode(true));
  REQUIRE_FALSE(hub.set_dehumidifier_mode(true));

  // Ensure nothing queued: pump loop and check mock TX has no write frames,
  // or expose pending_writes_len_ via TestableHub.
}

TEST_CASE("Allow thermostat writes with AWL thermostat", "[hub][awl][write]") {
  TestableHub hub;
  set_millis(0);
  drive_setup_thermostat(hub, 1, 300);
  REQUIRE(hub.set_heating_setpoint(70.0f));
  REQUIRE(hub.set_cooling_setpoint(74.0f));
  REQUIRE(hub.set_hvac_mode(HeatingMode::HEAT));
  REQUIRE(hub.set_fan_mode(FanMode::CONTINUOUS));
  // humidistat targets still false without humidifier — expect false:
  REQUIRE_FALSE(hub.set_humidification_target(40));
}
```

Add to `TestableHub`:

```cpp
size_t pending_writes() const { return this->pending_writes_len_; }
```

Assert `pending_writes() == 0` after rejected writes; `> 0` after accepted ones (before process).

- [ ] **Step 2: Run — expect FAIL (writes still accepted)**

```bash
cd tests && make test_hub && ./test_hub "[awl][write]"
```

- [ ] **Step 3: Gate write methods**

At the **top** of each method (before range checks), add capability check + WARN:

| Method | Gate |
| ------ | ---- |
| `set_heating_setpoint` | `awl_thermostat()` |
| `set_cooling_setpoint` | `awl_thermostat()` |
| `set_hvac_mode` | `awl_thermostat()` |
| `set_fan_mode` | `awl_thermostat()` |
| `set_fan_intermittent_on` | `awl_thermostat()` |
| `set_fan_intermittent_off` | `awl_thermostat()` |
| `set_humidification_target` | `has_humidistat_targets()` |
| `set_dehumidification_target` | `has_humidistat_targets()` |
| `set_humidifier_mode` | `has_humidistat_targets()` |
| `set_dehumidifier_mode` | `has_humidistat_targets()` |

Example:

```cpp
bool WaterFurnaceAurora::set_heating_setpoint(float temp) {
  if (!this->awl_thermostat()) {
    ESP_LOGW(TAG,
             "Ignoring heating setpoint write — no AWL communicating thermostat "
             "(dry-contact / Tstat v%.2f)",
             this->thermostat_version_);
    return false;
  }
  if (temp < 40.0f || temp > 90.0f) {
    ...
  }
  ...
}
```

Do **not** gate IZ2 zone setters.

- [ ] **Step 4: Run write tests — PASS**

```bash
cd tests && make test_hub && ./test_hub "[awl]"
```

- [ ] **Step 5: Commit**

```bash
git add components/waterfurnace_aurora/waterfurnace_aurora.cpp tests/test_hub.cpp
git commit -m "$(cat <<'EOF'
fix(hub): reject AWL thermostat and humidistat writes without capability

Dry-contact installs must not queue setpoint/mode/fan/humidistat target
writes that the wall tstat owns.
EOF
)"
```

---

### Task 3: Poll + publish gating and sanitization

**Files:**
- Modify: `components/waterfurnace_aurora/waterfurnace_aurora.cpp` (`build_poll_addresses_`, `publish_temperature_sensors_`, `publish_mode_sensors_`, `publish_humidity_control_sensors_`)
- Modify: `tests/test_hub.cpp`

- [ ] **Step 1: Failing publish/sanitize tests**

Need a way to run one publish cycle with injected register values after setup. Pattern used elsewhere: start poll, feed 0x42 response, or call a test hook.

Add to `TestableHub` if no cleaner path:

```cpp
void publish_from(const std::vector<std::pair<uint16_t, uint16_t>> &pairs) {
  RegisterMap regs;
  for (const auto &p : pairs) {
    reg_set(regs, p.first, p.second);
    reg_set(this->register_cache_, p.first, p.second);
  }
  this->publish_all_sensors_();  // uses register_cache_ — verify publish_all reads cache
}
```

**Check** `publish_all_sensors_()` — it may take `register_cache_` internally or last poll map. Read the function and match existing sensor tests (`Entering air temperature suppresses zero values`) which feed a poll response via mock UART. Prefer that proven pattern:

```cpp
// After drive_setup_thermostat(non-AWL):
// trigger update()/loop poll, feed response with HEATING_SETPOINT=1260, COOLING=600,
// HUMIDISTAT_TARGETS=0xFFFF, AMBIENT whatever, ENTERING_AIR=740 (74.0F)
// then REQUIRE isnan(get_heating_setpoint()) etc.
```

Study `TEST_CASE("Entering air temperature suppresses zero values")` and clone structure.

```cpp
TEST_CASE("Non-AWL publish ignores garbage setpoints and modes", "[hub][awl][publish]") {
  // non-AWL setup
  // poll response includes:
  //   HEATING_SETPOINT = 1260 (126.0 F), COOLING_SETPOINT = 600 (60.0 F)
  //   HEATING_MODE_READ packed heat, FAN_CONFIG, AMBIENT_TEMP garbage
  //   ENTERING_AIR = 740, SYSTEM_OUTPUTS with compressor+heat bits
  //   HUMIDISTAT_TARGETS = 0xFFFF
  REQUIRE(std::isnan(hub.get_heating_setpoint()));
  REQUIRE(std::isnan(hub.get_cooling_setpoint()));
  REQUIRE(std::isnan(hub.get_ambient_temperature()));
  // EAT cache should still update:
  REQUIRE(hub.get_entering_air_temperature() == Approx(74.0f));
  REQUIRE(hub.get_climate_current_temperature_f() == Approx(74.0f));
}

TEST_CASE("AWL publish accepts valid setpoints and rejects out-of-range", "[hub][awl][publish]") {
  // AWL setup, heating 700 (70F), cooling 740 (74F) → caches update
  // then heating 1260 → heating cache becomes NAN (or stays/forces NAN)
}

TEST_CASE("has_humidistat_targets gates humidistat target publish", "[hub][awl][publish]") {
  // AWL without humidifier: even if targets in response, sensors/caches don't take 255
  // non-AWL + humidifier: same
}
```

For RH: if value 255 present while somehow cached, force NAN when `> 100`.

- [ ] **Step 2: Run — FAIL on garbage still accepted**

```bash
cd tests && make test_hub && ./test_hub "[awl][publish]"
```

- [ ] **Step 3: Poll gating in `build_poll_addresses_`**

Replace unconditional ambient/setpoint/mode/fan adds:

```cpp
// Was always:
// add AMBIENT_TEMP, HEATING_SETPOINT, COOLING_SETPOINT, HEATING_MODE_READ, FAN_CONFIG

if (this->awl_thermostat()) {
  this->add_poll_addr_(registers::AMBIENT_TEMP);
  this->add_poll_addr_(registers::HEATING_SETPOINT);
  this->add_poll_addr_(registers::COOLING_SETPOINT);
  this->add_poll_addr_(registers::HEATING_MODE_READ);
  this->add_poll_addr_(registers::FAN_CONFIG);
}
```

Keep EAT (567/740) unconditional as today.

Replace humidistat medium-tier block:

```cpp
if (this->has_humidistat_targets()) {
  if (this->has_iz2_ && this->awl_communicating()) {
    this->add_poll_addr_(registers::IZ2_HUMIDISTAT_SETTINGS);
    this->add_poll_addr_(registers::IZ2_HUMIDISTAT_MODE);
    this->add_poll_addr_(registers::IZ2_HUMIDISTAT_TARGETS);
  } else {
    this->add_poll_addr_(registers::HUMIDISTAT_SETTINGS);
    this->add_poll_addr_(registers::HUMIDISTAT_TARGETS);
  }
}
```

Keep `RELATIVE_HUMIDITY` / `OUTDOOR_TEMP` on `awl_communicating()` as today.

- [ ] **Step 4: Publish gating + sanitizers**

**Ambient** (`publish_temperature_sensors_`):

```cpp
if (this->awl_thermostat()) {
  const uint16_t *val_amb = reg_find(regs, registers::AMBIENT_TEMP);
  if (val_amb) {
    this->ambient_temp_ = to_signed_tenths(*val_amb);
    ...
  }
} else {
  this->ambient_temp_ = NAN;
  // do not publish ambient sensor from 502
}
```

**Entering air** — also cache:

```cpp
if (val && *val != 0) {
  float fval = to_signed_tenths(*val);
  this->entering_air_temp_ = fval;
  ...
} 
// optional: if zero/missing, leave prior or set NAN — prefer set NAN when zero suppressed
```

**Setpoints** (inside cooldown window block):

```cpp
if (!this->awl_thermostat()) {
  this->heating_setpoint_ = NAN;
  this->cooling_setpoint_ = NAN;
} else if ((millis() - this->last_setpoint_write_) > WRITE_COOLDOWN_MS) {
  const uint16_t *val_hsp = reg_find(regs, registers::HEATING_SETPOINT);
  if (val_hsp) {
    float v = to_tenths(*val_hsp);
    if (std::isnan(v) || v < 40.0f || v > 90.0f) {
      this->heating_setpoint_ = NAN;
    } else {
      this->heating_setpoint_ = v;
      if (sensor_value_changed_(...)) publish...
    }
  }
  // cooling: valid 54–99 inclusive
  ...
}
```

When forcing NAN after previously published garbage, still call `publish_state(NAN)` if sensor configured and value changed (so HA clears bad state).

**RH:**

```cpp
if (val_rh) {
  float rh = static_cast<float>(*val_rh);
  if (rh > 100.0f) rh = NAN;  // includes 255
  this->relative_humidity_ = rh;
  ...
}
```

**Mode/fan** (`publish_mode_sensors_`): only update `hvac_mode_` / `fan_mode_` and text sensors when `awl_thermostat()`. When false, do not treat 12006/12005 as device truth (leave defaults; do not publish bogus mode strings from those regs).

**Humidistat targets** (`publish_humidity_control_sensors_`): wrap mode+target parsing in `if (this->has_humidistat_targets())`. Sanitize bytes:

```cpp
auto sane_hum = [](uint8_t b) -> float {
  if (b == 0 || b == 0xFF || b < 15 || b > 50) return NAN;
  return static_cast<float>(b);
};
auto sane_dehum = [](uint8_t b) -> float {
  if (b == 0 || b == 0xFF || b < 35 || b > 65) return NAN;
  return static_cast<float>(b);
};
```

Only publish sensor when non-NAN. Running binary sensors stay as today (no AWL gate).

- [ ] **Step 5: Run publish tests + full hub suite**

```bash
cd tests && make test_hub && ./test_hub
```

Expected: all PASS. Fix any test that assumed non-AWL setup still polled setpoints.

- [ ] **Step 6: Commit**

```bash
git add components/waterfurnace_aurora/waterfurnace_aurora.cpp tests/test_hub.cpp
git commit -m "$(cat <<'EOF'
fix(hub): gate poll/publish of thermostat and humidistat targets

Skip non-AWL setpoint/mode/ambient/humidistat registers, sanitize ranges,
cache entering air for climate fallback, and clear garbage to NAN.
EOF
)"
```

---

### Task 4: Climate entity status-oriented path

**Files:**
- Modify: `components/waterfurnace_aurora/climate/aurora_climate.cpp`
- Modify: `components/waterfurnace_aurora/climate/aurora_climate_utils.h`
- Modify: `tests/test_hub.cpp` and/or `tests/test_climate_math.cpp` only if harness can drive climate; otherwise hub helper coverage from Task 3 is enough and climate changes are thin.

- [ ] **Step 1: Update `read_mode_humidity_target`**

```cpp
inline float read_mode_humidity_target(WaterFurnaceAurora *hub, climate::ClimateMode mode) {
  if (hub == nullptr || !hub->has_humidistat_targets()) return NAN;
  ...
  uint8_t byte = ...;
  if (byte == 0 || byte == 0xFF) return NAN;
  // optional range check per mode
  return static_cast<float>(byte);
}
```

- [ ] **Step 2: Non-IZ2 `update_state_` branch**

Replace ambient-only current temp:

```cpp
float temp_f = this->parent_->get_climate_current_temperature_f();
if (!std::isnan(temp_f)) {
  float raw_c = fahrenheit_to_celsius(temp_f);
  this->current_temperature = apply_ema(raw_c, this->temp_ema_, EMA_ALPHA);
}
```

When `!this->parent_->has_awl_thermostat_controls()`:

```cpp
// Force unavailable targets so HA does not keep 126/60 or defaults forever
this->target_temperature_low = NAN;
this->target_temperature_high = NAN;
this->target_humidity = NAN;
// Do NOT update mode/preset/fan_mode from hub AWL regs
// Action block stays unchanged (outputs-based)
```

When `has_awl_thermostat_controls()`: keep existing setpoint/mode/fan update logic.

Shared humidity block already uses `read_mode_humidity_target` — with gate, non-AWL stays NAN. For non-AWL explicitly set `target_humidity = NAN` each update so stale optimistic control() values clear... Spec: control no-ops when hub returns false; still set NAN on update when gate false.

- [ ] **Step 3: Optional climate dump_config note**

```cpp
if (this->parent_->is_setup_complete() && !this->is_iz2_mode_()) {
  ESP_LOGCONFIG(TAG, "  AWL thermostat controls: %s",
                this->parent_->has_awl_thermostat_controls() ? "yes" : "no (status-only)");
}
```

- [ ] **Step 4: Build tests (climate compiles via hub only; no separate climate binary)**

```bash
cd tests && make test
```

If climate is not in unit test binary, ensure hub helper tests cover EAT fallback; manually review climate diff for `this->` style.

- [ ] **Step 5: Commit**

```bash
git add components/waterfurnace_aurora/climate/aurora_climate.cpp \
        components/waterfurnace_aurora/climate/aurora_climate_utils.h \
        tests/test_hub.cpp
git commit -m "$(cat <<'EOF'
fix(climate): status-only non-AWL path with entering-air fallback

Keep action and current temperature useful on dry-contact; clear bogus
targets/modes and ignore humidity targets without AWL humidistat path.
EOF
)"
```

---

### Task 5: Documentation

**Files:**
- Modify: `docs/ENTITIES.md`
- Modify: `README.md` (one bullet if climate features listed)

- [ ] **Step 1: ENTITIES.md** — under Climate Entities, after the main/IZ2 bullets:

```markdown
### Dry-contact / non-AWL thermostats

Installs **without** an AWL-compliant communicating thermostat or IntelliZone 2
(`Tstat` not installed or version &lt; 3.00) still show useful **status** on the
climate entity:

- **Action** tracks equipment from ABC outputs (heating/cooling/fan/idle)
- **Current temperature** uses entering/return air when ambient (register 502)
  is not valid for dry-contact
- **Target temperatures, HVAC mode, fan mode, and humidity targets** are not
  meaningful from holding registers and are left unavailable; writes are rejected

Full setpoint/mode/fan/humidity control requires a communicating AWL thermostat
or IZ2 zones. HA may still show climate controls (traits are static); values
stay unavailable and commands are ignored with a warning in the ESPHome log.
```

- [ ] **Step 2: README.md** — short bullet near climate/IZ2 notes:

```markdown
- Dry-contact (non-AWL) wall thermostats: climate is status-oriented (action +
  return-air temp); setpoints/mode writes require AWL thermostat or IZ2
```

- [ ] **Step 3: Commit**

```bash
git add docs/ENTITIES.md README.md
git commit -m "$(cat <<'EOF'
docs: note dry-contact / non-AWL climate behavior

Document status-only climate UX and write rejection without AWL/IZ2.
EOF
)"
```

---

### Task 6: Full verification + mark spec acceptance

- [ ] **Step 1: Full unit suite**

```bash
cd tests && make clean && make test
```

Expected: All tests passed (protocol, registers, hub, climate_math).

- [ ] **Step 2: Spec acceptance checklist** (tick in spec or PR body)

- Dry-contact: no garbage setpoints; climate targets NAN; humidity not 255  
- Dry-contact: `set_*` return false; no write frames  
- Dry-contact: action still works; current temp from EAT  
- AWL: setpoints/mode/fan/humidity still work when capable  
- `awl_thermostat()` = installed + version ≥ 3.0  
- `make test` green  
- ENTITIES/README note present  

- [ ] **Step 3: Update design spec status line**

```markdown
**Status:** Implemented (branch `fix/non-awl-climate-gating`)
```

Commit if desired with docs.

- [ ] **Step 4: Open PR** (from worktree)

```bash
git push -u origin HEAD
gh pr create --title "fix: non-AWL / dry-contact climate and setpoint gating" --body "$(cat <<'EOF'
## Summary
- Gate thermostat ambient/setpoints/mode/fan and humidistat targets on hub AWL capability (gem-compatible)
- Reject misleading writes on dry-contact installs
- Climate status-oriented path with entering-air current temp fallback
- Docs for dry-contact UX

Closes / relates to field report follow-up (GitHub #28 dry-contact targets).

## Spec
`docs/superpowers/specs/2026-07-24-non-awl-climate-gating-design.md`

## Test plan
- [x] `cd tests && make test`
- [ ] Field: dry-contact unit — no 126/60/255; action + EAT temp
- [ ] Field: AWL thermostat — regression on setpoints/mode/fan/humidity
EOF
)"
```

---

## Self-review (plan vs spec)

| Spec section | Task |
| ------------ | ---- |
| 5 Capability model | Task 1 |
| 6 Poll gating | Task 3 |
| 7 Publish gating | Task 3 |
| 8 Write gating | Task 2 |
| 9 Climate behavior | Task 4 |
| 10 Downstream entities | Inherited via hub (Tasks 2–3); no YAML |
| 11 Detection logs | Task 1 |
| 12 Unit tests | Tasks 1–4 |
| 13 Docs | Task 5 |
| 17 Acceptance | Task 6 |

No protocol changes. IZ2 zone control path unchanged except shared humidistat helpers.

---

## Execution notes for agents

1. Work only in `.worktrees/fix-non-awl-climate-gating`.
2. TDD: red → green → commit per task.
3. Always `this->` on members; no heap in `loop()`; no `mark_failed()` for this feature.
4. After Task 1 fixture fix, re-run **entire** `./test_hub` — outdoor temp and any AWL-assuming case must still pass.
5. Prefer public hub APIs for climate; keep climate thin.
6. Do not strip climate traits dynamically (spec non-goal).
