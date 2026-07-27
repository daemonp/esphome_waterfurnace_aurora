# Register Debug Access — Design Spec

**Date:** 2026-07-24  
**Status:** Draft (awaiting implementation plan)  
**Related:** GitHub issue #30 (open-loop field validation), closed-loop “Pump Type: Other” investigation  
**Implements surfaces:** HA service, result text sensor, Web UI entity pattern, opt-in HTTP last-result API

---

## 1. Problem

Field and maintainer debugging often needs a **one-shot read of an arbitrary Aurora register** without:

- Adding that register to the permanent poll set, or
- Flashing a special build, or
- Guessing from mapped entities alone

Concrete cases:

| Case | Need |
|------|------|
| Pump Type shows `Other` | Raw value of register **413** |
| Open-loop modulating output (#30) | Raw **321–325** before/without relying only on HA entities |
| Mystery NA / wrong scaling | Peek related regs while system is live |

### Current state

| Capability | Status |
|------------|--------|
| HA `write_register` service | Exists (`USE_API_CUSTOM_SERVICES`) |
| HA `read_register` service | **Missing** (docs that claim it exists are wrong) |
| Web UI register peek | **Missing** |
| HTTP register peek | **Missing** |
| One-shot read in hub state machine | **Missing** (only periodic poll + setup reads) |

---

## 2. Goals

1. **Read any single register (0–65535)** on demand via safe, async hub API.
2. Expose that capability through **four complementary surfaces** (phases below).
3. Always emit a clear **log line** on completion or failure.
4. Optionally show the last result in **HA / ESPHome Web UI** via a text sensor.
5. Preserve hub invariants: **no blocking UART in `loop()`**, no heap growth after setup, no `std::deque`, fixed-size pending state.
6. Default-safe: HTTP debug **off** unless explicitly enabled; **no HTTP write**.

## 3. Non-goals

- Replacing normal polled sensors / entity model
- Full address-space scan or continuous register streaming (future)
- HTTP or unauthenticated remote **write**
- Changing pump-type gating or open-loop poll policy (#30)
- Returning register values **inline** as HA service call results (HA custom services are fire-and-forget from the device’s perspective; use log + text sensor)

---

## 4. Shared core architecture

All surfaces call the same hub entry point.

```
  Surface 1: HA read_register(address)
  Surface 3: button / select / number → lambda
  Surface 4: HTTP queue + last-result GET
           │
           ▼
  bool request_debug_read(uint16_t addr)
           │  validate, rate-limit, single pending slot
           ▼
  Hub state machine performs one-shot FC3-style read
  (same framing as normal polls; one address)
           │
           ├── success → cache optional touch, format result,
           │             ESP_LOGI, publish text sensor (if any)
           └── fail    → ESP_LOGW, publish error string (if any)
```

### 4.1 Hub API

```cpp
/// Queue a one-shot read of a single register for debugging.
/// Returns false if address invalid, rate-limited, already pending, or hub not ready.
bool request_debug_read(uint16_t addr);
```

Internal state (fixed-size, no heap):

| Field | Purpose |
|-------|---------|
| `debug_read_pending_` | True while a debug read is queued or in flight |
| `debug_read_addr_` | Address requested |
| `debug_read_last_ms_` | For rate limiting |
| `debug_read_last_ok_` | Last completion success flag |
| `debug_read_last_addr_` | Last completed address |
| `debug_read_last_value_` | Last completed raw `uint16_t` (valid if ok) |
| `debug_read_last_error_` | Short static reason if !ok (`"timeout"`, `"busy"`, …) |

Optional: `text_sensor::TextSensor *debug_register_result_sensor_`

### 4.2 Scheduling rules

1. **Ready gate:** Reject (or defer—implementation choice: **reject with log** is simpler) if setup state machine has not finished hardware detection / first IDLE.
2. **Single slot:** If `debug_read_pending_`, return `false` and log once at WARN.
3. **Rate limit:** Minimum **500 ms** between accepted debug reads (constant `DEBUG_READ_COOLDOWN_MS`).
4. **Priority:** Debug read is lower priority than user **writes** already in `pending_writes_`, but may run at next IDLE poll opportunity (implementation: insert as a one-address poll burst or a dedicated state `DEBUG_READ_WAIT` after current transaction completes).
5. **Timeout:** Same response timeout as normal polls; on timeout clear pending and publish failure.
6. **No permanent poll change:** Debug read must **not** add the address to fast/medium/slow poll lists.

### 4.3 Result formatting

Log (success):

```text
[I][waterfurnace_aurora]: debug read: address=413 value=3 (0x0003)
```

Log (failure):

```text
[W][waterfurnace_aurora]: debug read: address=413 failed: timeout
```

Text sensor state (success):

```text
413 = 3 (0x0003)
```

Text sensor state (failure):

```text
413 = timeout
```

Keep strings short (ESPHome state size / mobile UI). Do **not** embed multi-line dumps.

### 4.4 Security / safety

- Reads are relatively safe; still rate-limit to avoid bus saturation.
- Do not expose writes on HTTP.
- Existing HA `write_register` remains the only intentional raw write path and already requires API + custom services.
- HTTP surface requires explicit YAML opt-in **and** inherits `web_server` auth when configured.

---

## 5. Surface 1 — HA API `read_register` service (Phase 1)

### Behavior

Under `#ifdef USE_API_CUSTOM_SERVICES`, register alongside `write_register`:

```cpp
register_service(&WaterFurnaceAurora::on_read_register_service_, "read_register",
                 {"address"});
```

Handler:

1. Validate `address` ∈ [0, 65535].
2. Call `request_debug_read(static_cast<uint16_t>(address))`.
3. Log acceptance or rejection immediately (`queued` vs `rejected: busy`).

### HA usage

```yaml
# Requires api with custom services enabled (same as write_register today)
service: esphome.<node_name>_read_register
data:
  address: 413
```

### Docs fix

`DEVELOPMENT.md` currently documents only `write_register`. Update to document `read_register` and remove any implication that read already existed.

### Acceptance

- [ ] Service appears when custom services compiled in  
- [ ] Calling with 413 produces success or failure log within one response timeout  
- [ ] Invalid address rejected without bus traffic  
- [ ] Rapid double-call: second rejected by pending/rate-limit  

---

## 6. Surface 2 — Last-result text sensor (Phase 2)

### Entity

Prefer consistency with other hub text sensors:

```yaml
text_sensor:
  - platform: waterfurnace_aurora
    aurora_id: aurora
    debug_register_result:
      name: "Debug Register Result"
```

### Behavior

- Unpublished / empty until first debug read completes (success or fail).
- Publish-on-change (same dedup pattern as other text sensors).
- Updated only from debug-read completion path (not from normal polls).

### Acceptance

- [ ] After HA `read_register`, sensor shows `413 = …`  
- [ ] Failure path updates sensor with error token  
- [ ] Entity optional: omitting it does not break Surface 1  

---

## 7. Surface 3 — Web UI via entities (Phase 3)

No custom HTML. Use stock ESPHome Web UI controls + hub API.

### Recommended YAML pattern (document in DEVELOPMENT.md / optional package comments)

```yaml
number:
  - platform: template
    name: "Debug Register Address"
    id: dbg_reg_addr
    min_value: 0
    max_value: 65535
    step: 1
    optimistic: true
    initial_value: 413
    mode: box

select:
  - platform: template
    name: "Debug Register Preset"
    id: dbg_reg_preset
    optimistic: true
    options:
      - "Pump Type (413)"
      - "VS Pump Min (321)"
      - "VS Pump Max (322)"
      - "VS Pump Manual (323)"
      - "VS Pump Speed (325)"
      - "Loop Pressure (1119)"
    initial_option: "Pump Type (413)"
    on_value:
      - lambda: |-
          // Map label → address; set dbg_reg_addr
          // (exact mapping table in implementation plan)

button:
  - platform: template
    name: "Read Debug Register"
    on_press:
      - lambda: |-
          id(aurora).request_debug_read(
              static_cast<uint16_t>(id(dbg_reg_addr).state));

text_sensor:
  - platform: waterfurnace_aurora
    aurora_id: aurora
    debug_register_result:
      name: "Debug Register Result"
```

### UX notes

| Control | Role |
|---------|------|
| Number (`mode: box`) | Freeform address |
| Select presets | Fast path for common regs |
| Button | Trigger read |
| Text sensor | Result on Web UI + HA |
| Device logs | Same result via `ESP_LOG*` |

Preset list is **documentation + example**, not exhaustive. Users may extend locally.

### Acceptance

- [ ] Documented snippet works with Web UI enabled  
- [ ] Preset changes address number  
- [ ] Button triggers same path as HA service  
- [ ] Result visible without attaching a serial log (via text sensor)  

---

## 8. Surface 4 — Opt-in HTTP last-result API (Phase 4)

**Model C (chosen):** simple curl discovery via **last completed result**, plus an explicit **queue** action. No long-wait inside the HTTP handler.

### Enablement

```yaml
waterfurnace_aurora:
  # ...
  debug_http: true   # default false
```

Requirements:

- `web_server` component present  
- `debug_http: true`  
- Handlers registered with normal web_server auth (`add_handler`, not `add_handler_without_auth`)

If `debug_http: true` without `web_server`, codegen/validation should **fail clearly** at config time.

### Endpoints

#### 8.1 Get last result (primary curl UX)

```http
GET /aurora/register
```

Response `200 application/json`:

```json
{
  "ok": true,
  "pending": false,
  "address": 413,
  "value": 3,
  "hex": "0x0003",
  "error": null,
  "age_ms": 1204
}
```

If never completed:

```json
{
  "ok": false,
  "pending": false,
  "address": null,
  "value": null,
  "hex": null,
  "error": "no_result_yet",
  "age_ms": null
}
```

If a read is in flight:

```json
{
  "ok": false,
  "pending": true,
  "address": 413,
  "value": null,
  "hex": null,
  "error": null,
  "age_ms": null
}
```

(`ok` refers to **last completed** read success when `pending` is false and a result exists.)

#### 8.2 Queue a read

```http
GET /aurora/register?addr=413
```

or (equivalent, preferred for clarity in docs):

```http
GET /aurora/register?addr=413&queue=1
```

Behavior:

1. Validate `addr`.  
2. Call `request_debug_read(addr)`.  
3. **Immediately** return JSON snapshot of **current** last-result + pending flag — **do not wait** for the bus.

Example just after queue accepted:

```json
{
  "ok": false,
  "pending": true,
  "queued": true,
  "address": 413,
  "value": null,
  "hex": null,
  "error": null,
  "age_ms": null
}
```

If queue rejected:

```json
{
  "ok": false,
  "pending": true,
  "queued": false,
  "reject_reason": "rate_limited",
  "address": 413,
  "value": null,
  "hex": null,
  "error": null,
  "age_ms": null
}
```

#### 8.3 Typical curl discovery flow

```bash
# 1) Queue
curl -s 'http://waterfurnace-aurora.local/aurora/register?addr=413'

# 2) Wait ~1s for bus

# 3) Read last result
curl -s 'http://waterfurnace-aurora.local/aurora/register'
```

Optional convenience (implementation may add later, not required for v1):

```http
GET /aurora/register?addr=413&wait_ms=500
```

**Out of scope for v1** — would approach model B. Spec leaves room but does not require it.

### Explicitly out of scope (HTTP)

- `POST` / `PUT` write  
- Bulk multi-register dump  
- WebSocket push of results  

### Acceptance

- [ ] Default build: no `/aurora/register` route  
- [ ] `debug_http: true` + web_server: both GET forms work  
- [ ] Auth honored when web_server auth configured  
- [ ] Handler never blocks on UART  

---

## 9. Phasing and dependencies

| Phase | Deliverable | Depends on |
|-------|-------------|------------|
| **P1** | `request_debug_read` + logs + HA `read_register` | — |
| **P2** | `debug_register_result` text sensor | P1 |
| **P3** | Documented Web UI YAML (number/select/button) | P1 + P2 |
| **P4** | Opt-in HTTP last-result + queue (model C) | P1 (P2 optional but recommended) |

**Minimum useful ship:** P1 alone unblocks raw 413 / 321–325 from HA logs.  
**Nice Web UI:** P1–P3.  
**Scriptable curl:** P4.

Suggested ship strategy: one PR for P1+P2, follow-up PR for P3 docs + P4, **or** single PR with P4 behind default-off flag if small enough.

---

## 10. Configuration summary

| Key | Where | Default | Notes |
|-----|--------|---------|-------|
| `api` + custom services | ESPHome | user config | Enables HA read/write services |
| `debug_register_result` | text_sensor platform | omitted | Surface 2 |
| `debug_http` | hub YAML | `false` | Surface 4 |
| Web UI templates | user YAML | omitted | Surface 3 examples in docs |

No Python codegen beyond: hub `debug_http` bool, text sensor key, validation vs `web_server`.

---

## 11. Testing plan

### Unit / hub tests (Catch2)

- `request_debug_read` rejects when pending  
- Rate limit rejects within cooldown  
- Successful path formats last_* fields  
- Failure path sets error token  
- Does not add address to poll address builder lists  

### Manual / field

1. HA `read_register` 413 on Series 7 closed loop → capture raw value for “Other” investigation.  
2. HA `read_register` 321–325 on open-loop test branch unit (#30).  
3. Web UI button path with text sensor visible.  
4. curl queue + last-result with `debug_http: true`.  
5. Confirm normal climate/sensor traffic unaffected under repeated debug reads at cooldown.

### Compile

- `tests/waterfurnace-test.yaml` remains green  
- Optional test fixture YAML snippet with debug entities (can be comments-only if entities need template platform)

---

## 12. Documentation updates (implementation time)

| File | Change |
|------|--------|
| `DEVELOPMENT.md` | Document `read_register`; Web UI snippet; HTTP curl flow; fix any “read already exists” wording |
| `README.md` | Short “Debug / advanced” pointer |
| `docs/ENTITIES.md` | `debug_register_result` row if shipped as platform entity |
| This spec | Status → Implemented + link plan/PR when done |

Optional: `docs/DEBUG_REGISTERS.md` how-to if DEVELOPMENT.md grows too large.

---

## 13. Risks and mitigations

| Risk | Mitigation |
|------|------------|
| Debug reads starve normal polls | Single slot + 500 ms cooldown; writes still prioritized |
| Users confuse number `0%` with real pump regs | Docs: sensors/NA vs debug raw values |
| HTTP on LAN without auth | Default `debug_http: false`; use web_server auth; read-only |
| Async confusion (“service returned but no value”) | Docs emphasize log + text sensor; HTTP model C makes pending explicit |
| Flash size | P4 optional; P1–P2 small |

---

## 14. Future extensions (not in this spec’s acceptance)

- Named register catalog in C++ shared by select presets and HTTP `?name=pump_type`  
- Multi-register debug read (small list, still one transaction if protocol allows)  
- `wait_ms` synchronous-ish HTTP helper  
- Integration with issue templates (“paste debug read of 413”)

---

## 15. Decision log

| Decision | Choice | Rationale |
|----------|--------|-----------|
| HA read service | Yes (P1) | Matches existing `write_register` |
| Result visibility | Log + optional text sensor | HA services don’t return values cleanly |
| Web UI | Entity pattern, not custom HTML | Stock Web UI, low maintenance |
| HTTP model | **C** — last result + queue | Simple curl discovery; no handler stall on bus |
| HTTP write | No | Writes already dangerous enough via HA |
| Default HTTP | Off | Least surprise / smaller attack surface |

---

## 16. Self-review checklist

- [x] No “TBD” placeholders for required behavior  
- [x] Phases ordered with clear dependencies  
- [x] Async bus constraints explicit  
- [x] HTTP model C specified with example JSON and curl flow  
- [x] Out of scope listed  
- [x] Ties to real field needs (#30, Pump Type Other / reg 413)  
- [x] ESPHome embedded constraints respected (fixed state, no blocking loop)  

---

## 17. Next step

After human approval of this spec file:

1. Run **writing-plans** → `docs/superpowers/plans/YYYY-MM-DD-register-debug-access.md`  
2. Implement P1→P2 first on a dedicated branch (not mixed with #30 open-loop field-test branch unless intentionally combined)  
)
