# Non-AWL / Dry-Contact Climate & Setpoint Gating — Design Spec

**Date:** 2026-07-24  
**Status:** Approved — implementation on `fix/non-awl-climate-gating`  
**Related:** GitHub #28 follow-up (daviss57 dry-contact climate targets); upstream gem `waterfurnace_aurora` **v1.6.4** at `../waterfurnace_aurora`  
**Decisions locked:** Scope = hub-wide (A); Climate UX = status-oriented (A); Writes = reject without AWL (A); Ambient = gate 502 + EAT fallback (B)

---

## 1. Problem

On installs **without** an AWL-compliant communicating thermostat or IZ2 (`Tstat: v0.00`, dry-contact wall tstat), the ABC still returns _something_ for thermostat/humidistat holding registers. Those values are not meaningful setpoints.

Field report (5-Series NSKW08H63ACCSSH, no AXB/VS/IZ2):

| Field                        | Observed                        | Meaning                            |
| ---------------------------- | ------------------------------- | ---------------------------------- |
| Climate targets              | Low 52.22 °C / High 15.56 °C    | 126 °F / 60 °F garbage; Low > High |
| Humidity setpoints           | 255                             | `0xFF` unpopulated byte            |
| Mode / action / live sensors | HEAT / HEATING / sensible temps | Bus and outputs path OK            |

RS-485 framing is fixed (#31/#33). This is **register semantics / UX**, not communications.

---

## 2. Upstream gem contract (v1.6.4 — verified)

Source: `../waterfurnace_aurora/lib/aurora/` (identical gating to prior `ccutre_wf` checkout for these paths).

### Capability flags

```ruby
def awl_thermostat?
  thermostat? && thermostat_version >= 3.0   # reg 800 != 3, reg 801 / 100
end

def awl_iz2?
  iz2? && iz2_version >= 2.0
end

def awl_communicating?
  awl_thermostat? || awl_iz2?
end
```

### What is read when

| Data                                                  | Registers                                  | Gate                                                                  |
| ----------------------------------------------------- | ------------------------------------------ | --------------------------------------------------------------------- |
| Ambient, heat/cool setpoints, fan config, target mode | 502, 745–746, 12005–12006                  | **`awl_thermostat?` only** (`Thermostat#registers_to_read`)           |
| Relative humidity, outdoor temp                       | 741–742                                    | **`awl_communicating?`** (`ABCClient` + humidistat)                   |
| Humidistat settings/targets                           | 12309–12310 or IZ2 equiv                   | **`awl_communicating?` AND** (humidifier OR dehumidifier OR VS drive) |
| Entering air, outputs, status, FP, etc.               | 30–31, 567/740, …                          | Always (core poll)                                                    |
| Call / “current mode” on thermostat object            | Derived from **inputs** Y1/Y2/O/G (reg 31) | Always refreshed; **not** from 12006 when non-AWL                     |

### Humidistat object lifetime

```ruby
@humidistat = nil unless humidifier? || dehumidifier? || awl_communicating?
```

Running flags still come from outputs/relays when the object exists; **targets are not refreshed** unless `awl_communicating?`.

### Writes in the gem

Setter methods (`heating_target_temperature=`, `target_mode=`, etc.) do **not** re-check AWL; the gem relies on not exposing UI when the component has no data. **Our ESPHome entities are always YAML-configured**, so we must **reject writes in the hub** when capability is absent (stricter than gem setters, correct for HA).

### IZ2

Per-zone ambient/setpoints/mode live on IZ2 registers and require `awl_iz2?` / zone setup. Unchanged by this work except shared humidistat gating already uses `awl_communicating()`.

---

## 3. Goals

1. **No bogus thermostat/humidistat targets** on non-AWL installs (climate, sensors, numbers, selects).
2. **Hub-wide gating** — one capability model; all entities inherit.
3. **Reject misleading writes** when the wall tstat owns dry-contact calls.
4. **Keep useful status** on dry-contact: action from outputs, current temperature via ambient **or** entering-air fallback, humidifier/dehumidifier running when hardware says so.
5. **Defensive publish filters** even when AWL is present (range / `0xFF`).
6. **Fix `awl_thermostat()`** to match gem (`thermostat installed && version >= 3.0`).
7. Unit tests + short docs note for dry-contact behavior.

## 4. Non-goals

- Turning HA into a full replacement thermostat over dry-contact (driving Y1/Y2).
- Dynamic climate **traits** stripping (HA caches `ListEntitiesClimateResponse`; fragile).
- Changing IZ2 zone math or VS-only paths beyond shared humidistat gates.
- Inferring a synthetic HA `climate.mode` from Y1/Y2 (optional later; see Open follow-ups).
- Removing YAML entities at codegen time based on runtime detection (detection is post-setup).

---

## 5. Capability model (hub)

### 5.1 Store thermostat installed

During hardware detection (already reads `THERMOSTAT_INSTALLED` = 800):

```cpp
// Match gem: thermostat? => holding_registers[800] != 3
this->has_thermostat_ = (*val != COMPONENT_NOT_INSTALLED);
```

Log in detection dump alongside Tstat version.

### 5.2 Fix helpers

```cpp
bool awl_thermostat() const {
  return this->has_thermostat_ && this->thermostat_version_ >= 3.0f;
}
bool awl_iz2() const {
  return this->has_iz2_ && this->iz2_version_ >= 2.0f;
}
bool awl_communicating() const {
  return this->awl_thermostat() || this->awl_iz2();
}
```

Optional convenience (documentation / call sites):

```cpp
/// True when system-wide (non-IZ2) AWL thermostat setpoint/mode/fan regs are valid.
bool has_awl_thermostat_controls() const { return this->awl_thermostat(); }

/// True when humidistat *target/mode registers* should be polled/published.
/// Gem: awl_communicating && (humidifier || dehumidifier || VS).
bool has_humidistat_targets() const {
  return this->awl_communicating() &&
         (this->has_humidifier_ || this->has_dehumidifier_ || this->has_vs_drive_);
}
```

Expose read-only accessors if tests/climate need them (`has_thermostat()`, existing version getters).

---

## 6. Poll gating (`build_poll_addresses_`)

### 6.1 Thermostat block (today always on fast tier)

| Register                    | Poll when          |
| --------------------------- | ------------------ |
| `AMBIENT_TEMP` (502)        | `awl_thermostat()` |
| `HEATING_SETPOINT` (745)    | `awl_thermostat()` |
| `COOLING_SETPOINT` (746)    | `awl_thermostat()` |
| `HEATING_MODE_READ` (12006) | `awl_thermostat()` |
| `FAN_CONFIG` (12005)        | `awl_thermostat()` |

Entering air (567/740) stays on the unconditional core list (supports climate temp fallback).

### 6.2 Humidity / outdoor (already partly correct)

| Register                  | Poll when                        |
| ------------------------- | -------------------------------- |
| `RELATIVE_HUMIDITY` (741) | `awl_communicating()` — **keep** |
| `OUTDOOR_TEMP` (742)      | `awl_communicating()` — **keep** |

### 6.3 Humidistat settings/targets (medium tier today)

Replace unconditional non-IZ2 poll with:

```
if (has_humidistat_targets()) {
  if (has_iz2_ && awl_communicating())
    poll IZ2_HUMIDISTAT_*;
  else
    poll HUMIDISTAT_SETTINGS + HUMIDISTAT_TARGETS;
}
```

Do **not** poll target regs when only a physical humidifier exists but there is no AWL path (gem agrees: `registers_to_read` empty without `awl_communicating?`).

### 6.4 Fan intermittent numbers

System-wide intermittent on/off numbers read `FAN_CONFIG`. When `FAN_CONFIG` is not polled, cache misses → NAN → numbers simply do not update (acceptable). Zone IZ2 paths unchanged.

---

## 7. Publish gating & sanitization

### 7.1 Setpoints

Only update `heating_setpoint_` / `cooling_setpoint_` (and sensors) when `awl_thermostat()` **and** value passes range:

| Value   | Valid range (°F) |
| ------- | ---------------- |
| Heating | 40–90 inclusive  |
| Cooling | 54–99 inclusive  |

Out of range or missing → leave cache as `NAN` (or set to `NAN` if previously garbage — prefer **force NAN when gate fails** so stale boot values cannot linger).

`to_tenths()` already maps `0x270F` → NAN; keep that.

### 7.2 HVAC mode / fan mode text sensors + caches

Only update `hvac_mode_` / `fan_mode_` from 12006/12005 when `awl_thermostat()`.

When gate is false: do **not** publish mode/fan text sensors from those regs; leave enums at defaults without claiming device truth. (Climate section defines entity behavior.)

### 7.3 Ambient

Only update `ambient_temp_` from 502 when `awl_thermostat()` and conversion is non-NAN.

### 7.4 Humidistat targets / auto modes

Only parse/publish humidification/dehumidification targets and auto/manual mode caches when `has_humidistat_targets()`.

Per-byte sanitization when publishing:

| Byte                                               | Valid                |
| -------------------------------------------------- | -------------------- |
| Humidification target                              | 15–50                |
| Dehumidification target                            | 35–65                |
| Either byte `0xFF` or `0x00` if treated as invalid | → NAN / skip publish |

(Confirm `0x00`: treat **0** as invalid for targets; gem ranges start at 15/35.)

Humidifier/dehumidifier **running** binary sensors stay gated only on DIP/hardware + outputs (no AWL required).

### 7.5 Relative humidity cache

Already polled only when `awl_communicating()`. Additionally ignore implausible values if desired (e.g. `> 100` or `255`) → NAN. Cheap safety.

---

## 8. Write gating (hub control API)

All of the following return `false` + `ESP_LOGW` when the gate fails **before** queueing a write:

| Method                                      | Gate                       |
| ------------------------------------------- | -------------------------- |
| `set_heating_setpoint`                      | `awl_thermostat()`         |
| `set_cooling_setpoint`                      | `awl_thermostat()`         |
| `set_hvac_mode`                             | `awl_thermostat()`         |
| `set_fan_mode`                              | `awl_thermostat()`         |
| `set_fan_intermittent_on/off` (system-wide) | `awl_thermostat()`         |
| `set_humidification_target`                 | `has_humidistat_targets()` |
| `set_dehumidification_target`               | `has_humidistat_targets()` |
| `set_humidifier_mode`                       | `has_humidistat_targets()` |
| `set_dehumidifier_mode`                     | `has_humidistat_targets()` |

IZ2 zone setters remain gated on IZ2 presence/zone validity as today (they are the AWL-IZ2 control path).

Keep existing numeric range checks **after** capability checks.

Log message pattern:

```text
Ignoring heating setpoint write — no AWL communicating thermostat (dry-contact / Tstat vX.XX)
```

---

## 9. Climate entity behavior (status-oriented)

Applies to the **non-IZ2 thermostat path** (`!is_iz2_mode_()`). IZ2 path unchanged except humidistat helpers already key off `awl_communicating()`.

### 9.1 Always (both AWL and dry-contact)

- **Action** from system outputs / lockout / dehumidify — unchanged.
- **Current temperature:**
  1. If `awl_thermostat()` and ambient valid → use ambient (°F → °C, existing EMA).
  2. Else if entering-air temperature valid (hub already suppresses raw 0) → use EAT as current temp with same EMA path.
  3. Else leave current temperature unset (`NAN`).
- **Current humidity:** only if RH cache non-NAN (implies AWL communicating + sane value).

### 9.2 When `awl_thermostat()` is true

- Publish `target_temperature_low/high` from heating/cooling setpoints (already sanitized).
- Publish `mode` / `preset` from hub HVAC mode.
- Publish `fan_mode` / custom intermittent from hub fan mode.
- Publish `target_humidity` via existing mode-aware helper when humidistat targets exist.
- `control()` writes flow to hub (hub still range-checks).

### 9.3 When `awl_thermostat()` is false (dry-contact)

| Climate field                          | Behavior                                                          |
| -------------------------------------- | ----------------------------------------------------------------- |
| `target_temperature_low/high`          | Do not update; ensure `NAN` so HA does not show 126/60            |
| `target_humidity`                      | Do not update; `NAN`                                              |
| `mode` / `preset`                      | Do **not** update from hub AWL mode regs                          |
| `fan_mode`                             | Do **not** update from hub fan regs                               |
| `action`                               | Still updated from outputs                                        |
| `current_temperature`                  | Ambient if AWL else EAT fallback                                  |
| `control()` mode/fan/setpoint/humidity | No-op after hub returns false; climate may log at DEBUG/WARN once |

**Traits:** leave static (still advertise two-point targets, modes, humidity). HA may show controls; values unavailable and writes rejected. Document this. Dynamic traits are a non-goal.

**Known UX quirk:** card may show default mode (e.g. OFF) while `action` is HEATING. Honest > pretty; optional follow-up can map call inputs to a read-only display mode.

### 9.4 Helper placement

Prefer small hub helpers used by climate:

```cpp
float get_climate_current_temperature_f() const;  // ambient if AWL tstat else EAT
bool has_awl_thermostat_controls() const;
```

Keep `this->` conventions; no heap; climate stays thin.

---

## 10. Downstream entities (inherit hub)

No special-case YAML required if hub publish/write gates are correct:

| Entity                                          | Non-AWL result                                   |
| ----------------------------------------------- | ------------------------------------------------ |
| Sensors `heating_setpoint`, `cooling_setpoint`  | Never get a state (or stay unavailable)          |
| Text `hvac_mode`, `fan_mode`                    | No AWL-driven updates                            |
| Sensors humidification/dehumidification targets | No updates without `has_humidistat_targets()`    |
| Numbers humidification/dehumidification         | `update_state_` sees NAN; `control` fails at hub |
| Numbers fan intermittent on/off (system)        | Cache miss without `FAN_CONFIG`                  |
| Humidistat selects                              | Mode cache not updated; `control` fails at hub   |
| Binary humidifier/dehumidifier running          | Still work when DIP/hardware says so             |

---

## 11. Detection / config logging

In setup complete / `dump_config` style logs, make capability obvious:

```text
Tstat: v0.00 installed=no awl_thermostat=no
AWL communicating: no (thermostat=no iz2=no)
Humidistat targets: no (need AWL + humidifier/dehumidifier/VS)
```

(Exact wording flexible; must include enough to debug another daviss57-style report.)

---

## 12. Testing

### 12.1 Unit tests (`tests/test_hub.cpp` and/or focused cases)

Simulate detection outcome without full UART where possible (set flags / drive setup fixtures):

1. **Non-AWL thermostat (version 0, not installed or installed+v0):**
   - `awl_thermostat() == false`
   - After a fake publish cycle with garbage 745=1260, 746=600, targets=0xFFFF:  
     `get_heating_setpoint()` / `get_cooling_setpoint()` are NAN
   - `set_heating_setpoint(70)` / `set_hvac_mode(...)` / `set_humidification_target(40)` return **false** and do not enqueue writes

2. **AWL thermostat (installed + version ≥ 3.00):**
   - Valid setpoints publish
   - Out-of-range 126 °F heating → NAN / not published
   - Writes still range-checked

3. **`has_humidistat_targets`:**
   - AWL + no hum/dehum/VS → false (no target poll side effects if testable)
   - AWL + humidifier → true
   - non-AWL + humidifier → false

4. **Ambient fallback helper:**
   - non-AWL + ambient NAN + EAT 74.0 → climate temp source 74.0
   - AWL + ambient 72 → 72 even if EAT differs

5. **`awl_thermostat` installed bit:** version 3.00 but `THERMOSTAT_INSTALLED == 3` → false

Climate math tests stay as-is; hub/climate integration tests only where the test harness already drives climate.

### 12.2 Manual / field

- daviss57-class dry-contact: no 126/60/255; action + current temp still populate.
- AWL thermostat install: regression — setpoints/mode/fan/humidity still work.

---

## 13. Documentation

- `docs/ENTITIES.md` — short **Dry-contact / non-AWL** subsection under Climate: status-only targets, writes ignored, needs IntelliZone or communicating thermostat for full control.
- `README.md` — one bullet if climate features are listed.
- Issue text can link this spec.

---

## 14. Implementation sketch (file touch list)

| File                                    | Change                                                                        |
| --------------------------------------- | ----------------------------------------------------------------------------- |
| `waterfurnace_aurora.h`                 | `has_thermostat_`; fix `awl_thermostat()`; helpers; maybe climate temp helper |
| `waterfurnace_aurora.cpp`               | Detection store; poll gates; publish gates + sanitizers; write gates; logs    |
| `climate/aurora_climate.cpp`            | Branch non-IZ2 update on `awl_thermostat()`; EAT fallback; NAN targets        |
| `climate/aurora_climate_utils.h`        | Humidity read already NAN-safe; ensure no 255 publish                         |
| `number/aurora_number.cpp`              | Optional early-out if desired; hub gate sufficient                            |
| `select/aurora_humidistat_select.cpp`   | Optional; hub gate sufficient                                                 |
| `tests/test_hub.cpp`                    | New sections above                                                            |
| `docs/ENTITIES.md` (+ README if needed) | User-facing note                                                              |

No protocol/register address changes.

---

## 15. Rollout / risk

| Risk                                                        | Mitigation                                    |
| ----------------------------------------------------------- | --------------------------------------------- |
| AWL user with odd firmware version &lt; 3.0 loses setpoints | Matches gem; version threshold is intentional |
| `has_thermostat_` false negative if reg 800 unread          | Already in detection address list             |
| Climate mode OFF + action HEATING looks odd                 | Documented; better than fake setpoints        |
| EAT used as room temp is return-air not zone temp           | Best available without AWL ambient; document  |
| Write reject surprises automations on dry-contact           | WARN log; correct behavior                    |

---

## 16. Open follow-ups (explicitly out of this spec)

1. Read-only climate `mode` derived from Y1/Y2/O inputs (gem `Thermostat#current_mode`).
2. Dynamic traits / disable climate controls in HA when non-AWL.
3. HA `available` flag on setpoint numbers when non-AWL.
4. Full virtual thermostat / equipment call control from ESPHome.

---

## 17. Acceptance criteria

- [ ] Dry-contact fixture: no heating/cooling setpoint sensor state from garbage regs; climate targets NAN; humidity target not 255
- [ ] Dry-contact: `set_*` thermostat/humidistat target APIs return false; no write frames queued
- [ ] Dry-contact: climate `action` still tracks equipment; `current_temperature` from EAT when ambient gated off
- [ ] AWL thermostat fixture: setpoints, mode, fan, humidity targets still poll/publish/write
- [ ] `awl_thermostat()` requires installed + version ≥ 3.0
- [ ] `cd tests && make test` green
- [ ] ENTITIES/README note present

---

## 18. Decision record

| #   | Topic               | Choice                                                          |
| --- | ------------------- | --------------------------------------------------------------- |
| 1   | Scope               | **A** Hub-wide gating                                           |
| 2   | Climate UX          | **A** Status-oriented (action + current temp; no bogus targets) |
| 3   | Writes without AWL  | **A** Reject all AWL thermostat + humidistat-target writes      |
| 4   | Ambient without AWL | **B** Gate 502; climate current temp falls back to entering air |
| —   | Gem reference       | `../waterfurnace_aurora` v1.6.4 re-verified 2026-07-24          |
