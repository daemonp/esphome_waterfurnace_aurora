# WaterFurnace Aurora for ESPHome

A native ESPHome component for controlling and monitoring WaterFurnace geothermal heat pumps. It communicates directly with the Aurora Base Control (ABC) board via RS-485 Modbus RTU, replacing Symphony/AWL hardware or Raspberry Pi bridges with a $5 ESP module.

An ESPHome/C++ port of the [waterfurnace_aurora Ruby gem](https://github.com/ccutrer/waterfurnace_aurora), optimized for ESP32/ESP8266 devices.

## Prerequisites

- **ESPHome 2026.1.x** or newer (the water heater platform requires 2026.1+)
- **Home Assistant 2024.1** or newer (for full entity support)

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
- DHW (Domestic Hot Water) as a native water heater entity (temperature, setpoint, on/off)
- Blower ECM speed settings (1-12)
- VS pump speed settings (1-100%)
- Fan intermittent timing
- Mode-aware humidity control (see [Humidity Control](#humidity-control) below)
- Humidifier and dehumidifier mode selection (Auto/Manual)
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

> **Important**: Use a MAX485 module with **exposed DE/RE pins**. Avoid "automatic flow control" modules (they lack the timing control needed for reliable Modbus RTU).

For detailed wiring diagrams, pin connections, RS485 module recommendations, and ESP8266-specific notes, see **[Hardware Setup & Wiring](docs/HARDWARE.md)**.

## Installation & Configuration

This component uses **ESPHome Packages** to pull the full sensor/control configuration from GitHub with a single import line. You only need to set your pin assignments and Wi-Fi credentials.

### Minimal Example

```yaml
substitutions:
  name: waterfurnace-aurora
  friendly_name: "Geothermal Heat Pump"
  # Adjust these three pins to match your wiring (see docs/HARDWARE.md)
  uart_tx_pin: GPIO17
  uart_rx_pin: GPIO16
  flow_control_pin: GPIO4

# This line pulls all component YAML (sensors, controls, climate, etc.)
# from GitHub automatically. No need to copy files into your config directory.
packages:
  waterfurnace.aurora: github://daemonp/esphome_waterfurnace_aurora/waterfurnace_aurora.yaml@master

esphome:
  name: ${name}
  friendly_name: ${friendly_name}

esp32:
  board: esp32dev
  framework:
    type: arduino

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
- **AXB Accessory Board**: register 806
- **VS Drive** (5/7-Series): ABC program name (registers 88-91) with fallback probe of VS-specific registers
- **IntelliZone 2**: register 812, zone count from register 483
- **Blower Type**: register 404 (PSC, ECM, 5-Speed); gates ECM speed controls
- **Energy Monitor Level**: register 412 (None, Compressor Monitor, Energy Monitor); gates refrigeration and energy registers
- **Pump Type**: register 413 (Open Loop, FC1, FC2, VS Pump, etc.); gates VS pump speed registers
- **AWL Versions**: thermostat (register 801), AXB (register 807), IZ2 (register 813); gates register selection

Sensors that depend on hardware not present show as "Unknown" in Home Assistant. If auto-detection fails, you can add manual overrides to your YAML. These merge with the package configuration (you do not need to redefine the base component):

```yaml
# Add this block alongside your packages: import to override auto-detection.
# The package already defines the waterfurnace_aurora component; ESPHome
# merges these keys with the package's base definition.
waterfurnace_aurora:
  has_axb: true
  has_vs_drive: true
  has_iz2: true
  num_iz2_zones: 3
```

### ESP8266 Notes

The ESP8266 is supported but you **must** add `logger: baud_rate: 0` to your YAML. The ESP8266 has only one hardware UART; without this setting the logger claims it and RS-485 communication will fail. See **[Hardware Setup](docs/HARDWARE.md#esp8266-notes)** for full details.

### IZ2 Zone Configuration

If you have an IntelliZone 2 system, **comment out the main "Heat Pump" climate entity** and uncomment zone entries matching your zone count. The main thermostat reads system-wide registers that contain IZ2 controller data, so it will show incorrect values (e.g., 32 F current temp, 158 F setpoints) when IZ2 is active.

```yaml
climate:
  # Comment out on IZ2 systems (shows incorrect values):
  # - platform: waterfurnace_aurora
  #   id: aurora_thermostat
  #   name: "Heat Pump"
  #   aurora_id: aurora

  # Uncomment zones matching your system (up to 6):
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
```

Each zone appears as a separate climate entity with its own temperature, setpoints, mode, and fan controls.

## Exposed Entities

The component supports 110+ entities across multiple platforms (not all are enabled in the default YAML):

| Category | Count | Examples |
| :--- | :--- | :--- |
| **Sensors** | ~60 | Temperatures, pressures, power, speeds, COP, water flow |
| **Binary Sensors** | 16 | Compressor, blower, aux heat, lockout, LPS/HPS, humidifier/dehumidifier |
| **Text Sensors** | 15 | Operating mode, fault/lockout diagnostics, model/serial, VS drive status |
| **Numbers** | 13 | ECM speeds, pump speeds, fan timing, humidity targets, line voltage |
| **Selects** | 2 | Humidifier mode, dehumidifier mode (Auto/Manual) |
| **Switches** | 2 | DHW enable (legacy), VS pump manual override |
| **Buttons** | 1 | Clear fault history |
| **Climate** | 1-7 | Main thermostat + up to 6 IZ2 zones |
| **Water Heater** | 1 | DHW with temperature, setpoint, and on/off mode |

For the complete list of every entity with registers and descriptions, see **[Exposed Entities](docs/ENTITIES.md)**.

## Humidity Control

The Aurora's humidistat manages two independent targets: a **humidification** setpoint (15-50%) for adding moisture and a **dehumidification** setpoint (35-65%) for removing it. The component routes these through the climate card's built-in humidity slider, so **no extra Home Assistant configuration is needed**.

### How It Works

The climate card's humidity slider automatically controls the right target based on the current HVAC mode:

| HVAC Mode | Slider Controls | Range |
| :--- | :--- | :--- |
| **Heat** / **Emergency Heat** / **Auto** | Humidification target | 15-50% |
| **Cool** | Dehumidification target | 35-65% |
| **Off** | Disabled (value preserved) | n/a |

When you switch from Heat to Cool, the slider value changes to reflect the dehumidification target (and vice versa). The two targets are stored independently (packed into a single 16-bit register), so adjusting one never affects the other.

On **IZ2 systems**, all zones share the same system-wide humidistat targets, but each zone's slider routes based on that zone's mode. Zone 1 in Heat shows the humidification target while Zone 2 in Cool shows the dehumidification target.

### Separate Humidifier / Dehumidifier Cards (Optional)

For users who prefer dedicated humidifier-domain cards in Lovelace (with target slider, mode selector, and on/off toggle), a ready-to-paste HA template configuration is provided:

```yaml
# In your HA configuration.yaml (copy from docs/ha_humidifier_templates.yaml).
# Replace "waterfurnace_aurora" with your ESPHome device name.
humidifier: !include ha_humidifier_templates.yaml
```

Or paste the contents of [`docs/ha_humidifier_templates.yaml`](docs/ha_humidifier_templates.yaml) directly into your `configuration.yaml`. This creates `humidifier.aurora_humidifier` and `humidifier.aurora_dehumidifier` entities backed by the same individual number/select/binary_sensor entities the component exposes.

See **[Exposed Entities: Humidity Control](docs/ENTITIES.md#humidity-control)** for the full list of underlying entities and register details.

## Troubleshooting

### No Communication
- Verify wiring (A/B polarity is often reversed; try swapping A and B)
- Check baud rate is 19200 with even parity
- Ensure the flow control pin is connected to DE/RE
- On ESP8266, confirm `logger: baud_rate: 0` is set (see [ESP8266 Notes](#esp8266-notes))

### Intermittent Errors
- Increase `read_retries` if you see occasional timeouts
- Check for loose connections
- Ensure adequate power supply to the RS485 module (ESP8266 is particularly sensitive to voltage drops during Wi-Fi + UART activity)

### Missing Sensors / "Unknown" Values
- Sensors show "Unknown" when the required hardware is not detected (AXB, VS Drive, IZ2)
- Check startup logs for detection messages (e.g., "AXB: detected", "VS Drive: detected")
- If auto-detection fails, use the YAML overrides described in [Hardware Detection](#hardware-detection)
- On IZ2 systems, see [IZ2 Zone Configuration](#iz2-zone-configuration) if the main thermostat shows bogus values

## Why Not ESPHome's Built-in Modbus Controller?

ESPHome ships with a [`modbus_controller`](https://esphome.io/components/modbus_controller) component that works well for standard Modbus devices. This component does not use it because the Aurora protocol is non-standard in several ways:

- **Proprietary function codes**: The ABC board uses custom function codes `0x42` (read a list of non-contiguous registers) and `0x41` (read multiple contiguous ranges). Neither is supported by `modbus_controller`. Function `0x42` fetches 50-100 scattered registers in a single transaction.

- **Bus efficiency**: Without `0x42`, you would need 15-30+ individual `0x03` reads per poll cycle to cover addresses spanning from `6` to `31460`. At 19200 baud that would overwhelm the ABC board and exceed its 100-register-per-operation limit.

- **Split read/write addresses**: The Aurora uses different registers for reading vs. writing the same parameter (e.g., heating setpoint is read from register `745` but written to `12619`). `modbus_controller` assumes a single address for both directions.

- **Three-tier adaptive polling**: This component polls fast-changing data every 5 s, configuration every 30 s, and fault history every 5 min. `modbus_controller` offers only a single `update_interval`.

For protocol details, see **[Protocol & Register Reference](docs/REGISTERS.md)**.

## Further Reading

| Document | Description |
| :--- | :--- |
| **[Hardware Setup & Wiring](docs/HARDWARE.md)** | RS485 module selection, wiring diagrams, ESP8266 notes |
| **[Exposed Entities](docs/ENTITIES.md)** | Complete list of all sensors, controls, and climate entities |
| **[Protocol & Register Reference](docs/REGISTERS.md)** | Modbus protocol details, adaptive polling, register map, fault codes |

## Community Projects

- [**Project Box for WaterFurnace Aurora**](https://github.com/benpeart/esphome_waterfurnace_aurora_projectbox) by [Ben Peart](https://github.com/benpeart): parts list, assembly instructions, and a 3D-printable project box for the ESP8266 + MAX485 module.

## Credits

Huge thanks to [Cody Cutrer (ccutrer)](https://github.com/ccutrer) for reverse engineering the protocol and creating the original Ruby implementation.

## License

This project is licensed under the MIT License.
