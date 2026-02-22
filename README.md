# WaterFurnace Aurora for ESPHome

A native ESPHome custom component for controlling and monitoring WaterFurnace geothermal heat pumps. This component communicates directly with the Aurora Base Control (ABC) board via RS-485 Modbus RTU, bypassing the need for expensive Symphony/AWL hardware or complex Raspberry Pi bridges.

It is a esphome/C++ port of the [waterfurnace_aurora Ruby gem](https://github.com/ccutrer/waterfurnace_aurora), optimized for ESP32/ESP8266 devices.

## Supported Systems

- **3-Series** (single-stage and two-stage compressors)
- **5-Series** (variable speed compressor)
- **7-Series** (variable speed compressor with enhanced monitoring)
- **IntelliZone 2 (IZ2)** multi-zone systems (up to 6 zones)
- **AXB Accessory Board** (for DHW, loop pump control, enhanced sensors)

## Features

### Climate Control
- Full thermostat integration (Heat, Cool, Auto, Emergency Heat modes)
- Fan modes (Auto, Continuous, Intermittent)
- Dual setpoint control (heating and cooling setpoints)
- IZ2 zone support with per-zone climate entities

### Monitoring
- **Temperatures**: Entering/leaving air, entering/leaving water, outdoor, ambient
- **Refrigeration**: Discharge/suction pressure, superheat, subcool, EEV position
- **VS Drive**: Compressor speed, fan speed, drive/inverter/ambient temperatures, power (5/7-Series)
- **VS Drive Diagnostics**: Derate status, safe mode flags, alarm codes (5/7-Series)
- **Energy**: Power consumption for compressor, blower, aux heat, loop pump
- **Derived Metrics**: COP (Coefficient of Performance), water delta-T, approach temperature
- **Loop**: Water flow rate, loop pressure
- **Humidity**: Relative humidity, humidification/dehumidification targets and modes
- **System Status**: LPS/HPS switches, lockout, emergency shutdown, load shed
- **Adaptive Polling**: Three-tier polling (fast/medium/slow) to reduce RS-485 bus load

### Controls
- DHW (Domestic Hot Water) enable/disable and setpoint
- Blower ECM speed settings (1-12)
- VS pump speed settings (1-100%)
- Fan intermittent timing
- Humidity targets
- Clear fault history

### Diagnostics
- Model number and serial number
- Fault code with human-readable descriptions
- Fault history (up to 10 past faults with descriptions)
- Current operating mode (Heating, Cooling, Standby, etc.)
- Anti-short-cycle countdown
- AXB DIP switch inputs
- Detected pump type

## Hardware Requirements

You need an **ESP32 or ESP8266**, a **MAX485 RS485 transceiver** with DE/RE flow control pins, and an **RJ45 cable** to connect to the heat pump's AID Tool port.

> **Important**: Use a MAX485 module with **exposed DE/RE pins**. Avoid "automatic flow control" modules — they lack the timing control needed for reliable Modbus RTU.

For detailed wiring diagrams, pin connections, RS485 module recommendations, and ESP8266-specific notes, see **[Hardware Setup & Wiring](docs/HARDWARE.md)**.

## Installation & Configuration

The recommended way to use this component is via **ESPHome Packages**. This allows you to include the full configuration with a single line and customize it using substitutions.

### Minimal Example

Add the following to your ESPHome YAML configuration:

```yaml
substitutions:
  name: waterfurnace-aurora
  friendly_name: "Geothermal Heat Pump"
  # Adjust pins to match your wiring
  uart_tx_pin: GPIO17
  uart_rx_pin: GPIO16
  flow_control_pin: GPIO4

packages:
  waterfurnace.aurora: github://daemonp/esphome_waterfurnace_aurora/waterfurnace_aurora.yaml@master

esphome:
  name: ${name}
  friendly_name: ${friendly_name}

esp32:
  board: esp32dev
  framework:
    type: arduino

# Standard ESPHome components
logger:
api:
ota:
wifi:
  ssid: "MyWiFi"
  password: "password"
```

### Configuration Variables

| Variable | Default | Description |
| :--- | :--- | :--- |
| `name` | - | Device name used for ESPHome ID |
| `friendly_name` | - | Friendly name for Home Assistant |
| `uart_tx_pin` | `GPIO17` | TX pin connected to RS485 module DI |
| `uart_rx_pin` | `GPIO16` | RX pin connected to RS485 module RO |
| `flow_control_pin` | `GPIO4` | Pin for RS485 DE/RE flow control |
| `modbus_address` | `1` | Modbus slave address of the Aurora ABC board |
| `update_interval` | `5s` | How often to poll the heat pump (base interval for fast tier) |
| `read_retries` | `2` | Number of Modbus read retries on failure (0-10) |

### Hardware Detection

The component automatically detects installed hardware at startup:
- **AXB Accessory Board** — detected via register 806
- **VS Drive** (5/7-Series) — detected via ABC program name (registers 88-91 for "VSP"/"SPLVS") with fallback probe of VS-specific registers (3001, 3322, 3325)
- **IntelliZone 2** — detected via register 812, zone count from register 483
- **Blower Type** — detected via register 404 (PSC, ECM 208/230, ECM 265/277, 5-Speed); gates ECM speed controls
- **Energy Monitor Level** — detected via register 412 (None, Compressor Monitor, Energy Monitor); gates refrigeration and energy registers
- **Pump Type** — detected via register 413 (Open Loop, FC1, FC2, VS Pump, etc.); gates VS pump speed registers
- **AWL Versions** — thermostat (register 801), AXB (register 807), IZ2 (register 813); gates register selection (e.g., AWL AXB >= 2.0 uses different entering air registers, AWL thermostat >= 3.0 enables humidity/outdoor temp)

Sensors that depend on hardware not present will show as "Unknown" in Home Assistant. If auto-detection fails or gives incorrect results, you can manually override these settings in the `waterfurnace_aurora` component configuration:

```yaml
waterfurnace_aurora:
  id: aurora
  uart_id: mod_bus
  address: 1
  flow_control_pin: GPIO4
  # Manual hardware overrides (auto-detected if omitted)
  has_axb: true
  has_vs_drive: true
  has_iz2: true
  num_iz2_zones: 3
```

### ESP8266 Notes

The ESP8266 is supported but you **must** set `logger: baud_rate: 0` to free the UART for RS-485. See **[Hardware Setup](docs/HARDWARE.md#esp8266-notes)** for full details.

### IZ2 Zone Configuration

If you have an IntelliZone 2 system, you should:

1. **Comment out** (or remove) the main "Heat Pump" climate entity — on IZ2 systems, it reads system-wide registers that contain IZ2 controller data, resulting in incorrect temperature values.
2. **Uncomment** the zone climate entries matching your zone count.

```yaml
climate:
  # IMPORTANT: Comment out the main thermostat on IZ2 systems!
  # It reads system-wide registers that show incorrect values when IZ2 is active.
  # - platform: waterfurnace_aurora
  #   id: aurora_thermostat
  #   name: "Heat Pump"
  #   aurora_id: aurora

  # IZ2 Zone climates - uncomment zones matching your system
  - platform: waterfurnace_aurora
    id: aurora_zone_1
    name: "Zone 1"
    aurora_id: aurora
    zone: 1
  - platform: waterfurnace_aurora
    id: aurora_zone_2
    name: "Zone 2"
    aurora_id: aurora
    zone: 2
  - platform: waterfurnace_aurora
    id: aurora_zone_3
    name: "Zone 3"
    aurora_id: aurora
    zone: 3
  # ... up to zone 6
```

The component auto-detects IZ2 and the zone count. Each zone appears as a separate climate entity in Home Assistant with its own temperature, setpoints, mode, and fan controls.

## Exposed Entities

The component creates 50+ sensors, 12 binary sensors, 14 text sensors, 11 number controls, switches, buttons, and climate entities. Here's a summary:

| Category | Count | Examples |
| :--- | :--- | :--- |
| **Sensors** | ~55 | Temperatures, pressures, power, speeds, COP, water flow |
| **Binary Sensors** | 12 | Compressor, blower, aux heat, lockout, LPS/HPS |
| **Text Sensors** | 14 | Operating mode, fault description, model/serial, VS drive status |
| **Numbers** | 11 | DHW setpoint, ECM speeds, pump speeds, fan timing, humidity targets |
| **Switches** | 1 | DHW enable/disable |
| **Buttons** | 1 | Clear fault history |
| **Climate** | 1-7 | Main thermostat + up to 6 IZ2 zones |

For the complete list of every entity with registers and descriptions, see **[Exposed Entities](docs/ENTITIES.md)**.

## Humidistat Setup (Humidifier / Dehumidifier Cards)

The Aurora's humidistat has two independent targets — humidification (15-50%) and dehumidification (35-65%) — both always active. ESPHome does not have a native `humidifier` platform, so the component exposes humidistat controls as individual `number`, `select`, and `binary_sensor` entities.

To get proper **humidifier and dehumidifier cards** in Lovelace (with target slider, mode selector, and on/off toggle), add the provided Home Assistant template configuration to your `configuration.yaml`:

```yaml
# In your HA configuration.yaml — copy from docs/ha_humidifier_templates.yaml
# Replace "waterfurnace_aurora" with your ESPHome device name
humidifier: !include ha_humidifier_templates.yaml
```

Or paste the contents of [`docs/ha_humidifier_templates.yaml`](docs/ha_humidifier_templates.yaml) directly into your `configuration.yaml`.

This creates two entities:
- **`humidifier.aurora_humidifier`** — adds moisture when RH drops below target
- **`humidifier.aurora_dehumidifier`** — removes moisture when RH rises above target

See **[Exposed Entities — Humidistat](docs/ENTITIES.md#humidistat-humidifier--dehumidifier-cards)** for details on the underlying ESPHome entities.

## Troubleshooting

### No Communication
- Verify wiring (A/B polarity is often reversed)
- Check baud rate is 19200 with even parity
- Ensure flow control pin is connected to DE/RE
- Try swapping A and B wires

### Intermittent Errors
- Increase `read_retries` if you see occasional timeouts
- Check for loose connections
- Ensure adequate power supply to RS485 module

### Missing Sensors / "Unknown" Values
- Sensors show "Unknown" when the required hardware is not detected (AXB, VS Drive, IZ2)
- Check logs at startup for hardware detection messages (e.g., "AXB: detected", "VS Drive: detected", "IZ2: detected")
- If auto-detection fails, use the `has_axb`, `has_vs_drive`, `has_iz2`, and `num_iz2_zones` YAML overrides
- On IZ2 systems, the main "Heat Pump" climate entity will show bogus values (e.g., 32°F current temp, 158°F setpoints) — use zone climate entities instead

### IZ2 Climate Shows Incorrect Values
- If you have an IntelliZone 2 system and the main thermostat shows wrong temperatures (e.g., 32°F, 158°F), this is because the system-wide registers contain IZ2 controller data
- **Solution**: Comment out the main "Heat Pump" climate entity and use zone-specific climate entities (zone: 1, zone: 2, etc.)

### ESP8266: No Communication or Watchdog Resets
- Set `logger: baud_rate: 0` — the ESP8266 has only one UART and the logger will conflict with RS-485 communication
- Ensure adequate power supply — the ESP8266 can be sensitive to voltage drops during WiFi + UART activity

## Why Not ESPHome's Built-in Modbus Controller?

ESPHome ships with a [`modbus_controller`](https://esphome.io/components/modbus_controller) component that works great for standard Modbus devices. This component doesn't use it because the WaterFurnace Aurora protocol isn't standard Modbus:

- **Proprietary function codes** — The Aurora ABC board supports custom function codes `0x42` (read a list of arbitrary non-contiguous registers) and `0x41` (read multiple contiguous ranges), neither of which `modbus_controller` supports. Function `0x42` is the primary read method: it fetches 50-100 scattered registers in a single transaction.

- **Bus efficiency** — Without `0x42`, you'd need 15-30+ individual `0x03` reads per poll cycle to cover addresses spanning from `6` to `31460`. At 19200 baud that would overwhelm the ABC board and blow past its 100-register-per-operation limit.

- **Split read/write addresses** — The Aurora uses different registers for reading vs. writing the same parameter (e.g., heating setpoint is read from register `745` but written to `12619`). The `modbus_controller` Number/Select/Switch components assume a single address for both directions.

- **Three-tier adaptive polling** — The component polls fast-changing data every 5 s, configuration data every 30 s, and fault history every 5 min. `modbus_controller` offers only a single `update_interval` per controller.

In short, the Aurora's proprietary extensions make a custom Modbus stack the only practical approach. For protocol details, see **[Protocol & Register Reference](docs/REGISTERS.md)**.

## Further Reading

| Document | Description |
| :--- | :--- |
| **[Hardware Setup & Wiring](docs/HARDWARE.md)** | RS485 module selection, wiring diagrams, ESP8266 notes |
| **[Exposed Entities](docs/ENTITIES.md)** | Complete list of all sensors, controls, and climate entities |
| **[Protocol & Register Reference](docs/REGISTERS.md)** | Modbus protocol details, adaptive polling, register map, fault codes |

## Community Projects

- [**Project Box for WaterFurnace Aurora**](https://github.com/benpeart/esphome_waterfurnace_aurora_projectbox) by [Ben Peart](https://github.com/benpeart) — Parts list, assembly instructions, and a 3D-printable project box to house the ESP8266 + MAX485 module.

## Credits

Huge thanks to [Cody Cutrer (ccutrer)](https://github.com/ccutrer) for reverse engineering the protocol and creating the original Ruby implementation.

## License

This project is licensed under the MIT License.
