# WaterFurnace Aurora for ESPHome

A native ESPHome custom component for controlling and monitoring WaterFurnace geothermal heat pumps. This component communicates directly with the Aurora Base Control (ABC) board via RS-485 Modbus RTU, bypassing the need for expensive Symphony/AWL hardware or complex Raspberry Pi bridges.

It is a C++ port of the [waterfurnace_aurora Ruby gem](https://github.com/ccutrer/waterfurnace_aurora), optimized for ESP32/ESP8266 devices.

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

1. **ESP32 or ESP8266 Development Board** (e.g., Wemos D1 Mini, NodeMCU, ESP32-DevKit)
2. **TTL to RS485 Adapter** with manual DE/RE flow control pins (see note below)
3. **RJ45 Connector/Cable**: To connect to the AID Tool port on the heat pump

### Recommended RS485 Module

Use a MAX485 module with **exposed DE/RE pins** for manual flow control. A known working module is the [Alinan MAX485 RS485 Transceiver Module](https://www.amazon.com/dp/B00NIOLNAG).

> **Important**: Avoid RS485 modules with "automatic flow control" that only have VCC/TXD/RXD/GND pins. These lack the DE/RE pins needed for reliable Modbus RTU timing and will cause communication errors.

### Wiring

Connect the RS485 adapter to the ESP and the Heat Pump's AID Tool port (RJ45).

| Heat Pump (RJ45) | Wire Color (T568B) | RS485 Adapter | ESP32/8266 |
| :--- | :--- | :--- | :--- |
| Pin 1 (RS485+) | White/Orange | A / + | - |
| Pin 2 (RS485-) | Orange | B / - | - |
| Pin 3 (RS485+) | White/Green | A / + | - |
| Pin 4 (RS485-) | Blue | B / - | - |
| - | - | DI (TX) | GPIO_TX |
| - | - | RO (RX) | GPIO_RX |
| - | - | DE/RE | GPIO_FLOW |
| - | - | VCC | 3.3V / 5V |
| - | - | GND | GND |

> **Note**: The DE/RE pins on most RS485 modules need to be tied together and connected to a GPIO for flow control. The component automatically switches this pin HIGH during transmit and LOW during receive.

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

The component automatically detects the installed hardware at startup:
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

The ESP8266 (including D1 Mini / NodeMCU) is supported but requires special configuration:

1. **Logger baud rate**: The ESP8266 has only one fully functional UART. You **must** set `logger: baud_rate: 0` to disable serial logging so the UART is available for RS-485 communication:

   ```yaml
   logger:
     baud_rate: 0
   ```

2. **Pin selection**: The default GPIO pins (16/17) are ESP32 pins. For ESP8266, use appropriate pins (e.g., GPIO1/GPIO3 for hardware UART, or software serial pins).

3. **Memory**: The ESP8266 has limited RAM. If you experience stability issues, consider disabling sensors you don't need by commenting them out in your YAML.

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

### Sensors

| Sensor | Unit | Description | Register |
| :--- | :--- | :--- | :--- |
| `entering_air_temperature` | °F | Air temperature entering the unit | 567/740 |
| `leaving_air_temperature` | °F | Air temperature leaving the unit | 900 |
| `ambient_temperature` | °F | Room/zone temperature | 502 |
| `outdoor_temperature` | °F | Outdoor temperature (AWL) | 742 |
| `entering_water_temperature` | °F | Loop water entering the unit | 1111 |
| `leaving_water_temperature` | °F | Loop water leaving the unit | 1110 |
| `heating_setpoint` | °F | Current heating setpoint | 745 |
| `cooling_setpoint` | °F | Current cooling setpoint | 746 |
| `humidity` | % | Relative humidity | 741 |
| `compressor_speed` | RPM | Actual compressor speed (VS) | 3001 |
| `compressor_desired_speed` | RPM | Target compressor speed (VS) | 3000 |
| `discharge_pressure` | psi | Refrigerant discharge pressure | 3322 |
| `suction_pressure` | psi | Refrigerant suction pressure | 3323 |
| `superheat_temperature` | °F | Superheat temperature | 3906 |
| `eev_open_percentage` | % | Electronic expansion valve opening | 3808 |
| `line_voltage` | V | Incoming line voltage | 16 |
| `total_watts` | W | Total system power consumption | 1152-1153 |
| `compressor_watts` | W | Compressor power consumption | 1146-1147 |
| `blower_watts` | W | Blower power consumption | 1148-1149 |
| `aux_heat_watts` | W | Auxiliary heat power consumption | 1150-1151 |
| `pump_watts` | W | Loop pump power consumption | 1164-1165 |
| `waterflow` | GPM | Loop water flow rate | 1117 |
| `loop_pressure` | psi | Loop pressure | 1119 |
| `dhw_temperature` | °F | Domestic hot water temperature | 1114 |
| `dhw_setpoint` | °F | DHW setpoint | 401 |
| `fault_code` | - | Current/last fault code | 25 |
| `fp1_temperature` | °F | Freeze protection sensor 1 (liquid line) | 19 |
| `fp2_temperature` | °F | Freeze protection sensor 2 (air coil) | 20 |
| `anti_short_cycle` | s | Anti-short-cycle countdown | 6 |
| `discharge_temperature` | °F | Compressor discharge temperature | 3325 |
| `suction_temperature` | °F | Compressor suction temperature | 3903 |
| `vs_drive_temperature` | °F | VS drive board temperature | 3327 |
| `vs_inverter_temperature` | °F | VS inverter temperature | 3522 |
| `blower_speed` | - | Current ECM blower speed | 344 |
| `pump_speed` | % | Current VS pump speed | 325 |
| `heating_liquid_line_temperature` | °F | Heating mode liquid line temp | 1109 |
| `saturated_condenser_temperature` | °F | Saturated condenser temperature | 1134 |
| `subcool_temperature` | °F | Subcooling temperature | 1135/1136 |
| `heat_of_extraction` | BTU/h | Heat extracted from loop | 1154-1155 |
| `heat_of_rejection` | BTU/h | Heat rejected to loop | 1156-1157 |
| `cop` | - | Coefficient of Performance (heating or cooling) | Derived |
| `water_delta_t` | °F | Water temperature delta (leaving - entering) | Derived |
| `approach_temperature` | °F | Heat exchanger approach temperature | Derived |
| `vs_fan_speed` | % | VS drive fan speed (5/7-Series) | 3524 |
| `vs_ambient_temperature` | °F | VS compressor ambient temperature (5/7-Series) | 3326 |
| `vs_compressor_watts` | W | VS drive compressor power (5/7-Series) | 3422-3423 |
| `saturated_evaporator_discharge_temperature` | °F | Saturated evaporator discharge temp | 3905 |
| `aux_heat_stage` | - | Aux heat stage (0=off, 1=EH1, 2=EH1+EH2) | Derived (reg 30) |
| `blower_only_speed` | - | Blower-only ECM speed setting | 340 |
| `lo_compressor_speed` | - | Low compressor ECM speed setting | 341 |
| `hi_compressor_speed` | - | High compressor ECM speed setting | 342 |
| `aux_heat_speed` | - | Aux heat ECM speed setting | 347 |
| `pump_min_speed` | % | VS pump minimum speed setting | 321 |
| `pump_max_speed` | % | VS pump maximum speed setting | 322 |
| `humidification_target` | % | Humidification target | 12310/31110 |
| `dehumidification_target` | % | Dehumidification target | 12310/31110 |
| `line_voltage_setting` | V | Configured line voltage setting | 112 |

#### Derived Sensor Details

**COP (Coefficient of Performance)** — Computed on-device each cycle:
- **Heating mode**: `Heat_of_Rejection / (Total_Watts * 3.412)`
- **Cooling mode**: `Heat_of_Extraction / (Total_Watts * 3.412)`
- Returns `0.0` when the compressor is off. Values are clamped to the 0.5–15.0 range; outliers are rejected as sensor noise.

**Subcool Temperature** — Automatically selects the correct register based on the current operating mode: register 1135 during heating, register 1136 during cooling (based on reversing valve state).

**Approach Temperature** — Heat exchanger approach:
- **Heating mode**: `Leaving Water - Saturated Condenser`
- **Cooling mode**: `Saturated Condenser - Entering Water`

### Binary Sensors

| Sensor | Description | Source |
| :--- | :--- | :--- |
| `compressor_running` | Compressor is running | Register 30 (CC/CC2 bits) |
| `blower_running` | Blower is running | Register 30 (BLOWER bit) |
| `aux_heat_running` | Auxiliary heat is active | Register 30 (EH1/EH2 bits) |
| `dhw_running` | DHW mode is active | Register 1104 (DHW bit) |
| `loop_pump_running` | Loop pump is running | Register 1104 (LOOP_PUMP bit) |
| `lockout` | System is in lockout | Register 25 (bit 15) |
| `low_pressure_switch` | LPS has tripped | Register 31 (bit 7) |
| `high_pressure_switch` | HPS has tripped | Register 31 (bit 8) |
| `emergency_shutdown` | Emergency shutdown active | Register 31 (bit 6) |
| `load_shed` | Load shed/demand response active | Register 31 (bit 9) |
| `humidifier_running` | Humidifier is running | Register 30 (ACCESSORY bit) |
| `dehumidifier_running` | Dehumidifier/active dehum | Register 362 / 1104 |

### Text Sensors

| Sensor | Description |
| :--- | :--- |
| `current_mode` | Current operating mode (see values below) |
| `hvac_mode` | Configured HVAC mode (Off, Auto, Cool, Heat, E-Heat) |
| `fan_mode` | Configured fan mode (Auto, Continuous, Intermittent) |
| `fault_description` | Human-readable fault description |
| `fault_history` | Up to 10 past faults with descriptions (semicolon-separated) |
| `model_number` | Heat pump model number (registers 92-103) |
| `serial_number` | Heat pump serial number (registers 105-109) |
| `vs_derate` | VS Drive derate status flags (5/7-Series) |
| `vs_safe_mode` | VS Drive safe mode status flags (5/7-Series) |
| `vs_alarm` | VS Drive alarm codes (5/7-Series) |
| `axb_inputs` | AXB DIP switch input states |
| `humidifier_mode` | Humidifier mode (Auto or Manual) |
| `dehumidifier_mode` | Dehumidifier mode (Auto or Manual) |
| `pump_type` | Detected pump type (e.g., "VS Pump", "FC1", "Open Loop") |

#### `current_mode` Values

The `current_mode` text sensor reports one of the following values:

| Value | Description |
| :--- | :--- |
| `Standby` | System idle, no demand |
| `Heating` | Compressor running in heating mode |
| `Cooling` | Compressor running in cooling mode |
| `Heating + Aux` | Heating with auxiliary electric heat |
| `Emergency Heat` | Electric heat only, compressor locked out |
| `Dehumidify` | Active dehumidification |
| `Fan Only` | Blower running, no compressor |
| `Waiting` | Anti-short-cycle countdown active |
| `Lockout` | System locked out due to fault |

### Controls (Numbers)

| Control | Range | Description |
| :--- | :--- | :--- |
| `dhw_setpoint` | 100-140°F | DHW temperature setpoint |
| `blower_only_speed` | 1-12 | ECM blower speed for fan-only mode |
| `lo_compressor_speed` | 1-12 | ECM blower speed for low compressor |
| `hi_compressor_speed` | 1-12 | ECM blower speed for high compressor |
| `aux_heat_speed` | 1-12 | ECM blower speed for aux heat |
| `pump_min_speed` | 1-100% | VS pump minimum speed |
| `pump_max_speed` | 1-100% | VS pump maximum speed |
| `fan_intermittent_on` | 0-25 min | Fan on time (intermittent mode) |
| `fan_intermittent_off` | 5-40 min | Fan off time (intermittent mode) |
| `humidification_target` | 15-50% | Humidification setpoint |
| `dehumidification_target` | 35-65% | Dehumidification setpoint |

### Switches

| Switch | Description |
| :--- | :--- |
| `dhw_enabled` | Enable/disable domestic hot water |

### Buttons

| Button | Description |
| :--- | :--- |
| `clear_fault_history` | Clear the fault history log |

## Protocol Details

This component implements a custom Modbus client. Standard Modbus libraries cannot read the Aurora registers efficiently because WaterFurnace uses proprietary function codes:

| Function Code | Name | Description |
| :--- | :--- | :--- |
| `0x03` | Read Holding Registers | Standard Modbus read (used for setup) |
| `0x06` | Write Single Register | Standard Modbus write |
| `0x41` ('A') | Read Ranges | Read multiple register ranges in one request |
| `0x42` ('B') | Read Specific | Read arbitrary non-sequential registers |

The custom function codes allow fetching 50+ data points in just a few transactions, keeping the bus responsive and reducing latency.

### Communication Settings

- **Baud Rate**: 19200
- **Parity**: Even
- **Stop Bits**: 1
- **Default Address**: 1

### Read Retry Logic

The component implements automatic retry logic (default 2 retries) for communication failures, matching the behavior of the Ruby gem. This can be configured via the `read_retries` parameter.

### Adaptive Polling

To reduce RS-485 bus load and improve responsiveness, the component uses three-tier adaptive polling. The `update_interval` setting controls the base (fast) tier; the other tiers are multiples of it.

| Tier | Default Interval | Data Polled |
| :--- | :--- | :--- |
| **Fast** | Every cycle (~5s) | Temperatures, compressor speed, power consumption, VS drive telemetry, water flow, blower/pump speed, IZ2 zone data |
| **Medium** | Every 6th cycle (~30s) | Setpoints, mode config, DHW settings, line voltage, ECM/pump speed settings, VS drive status flags (derate/safe/alarm), humidistat settings, AXB inputs, loop pressure |
| **Slow** | Every 60th cycle (~5 min) | Fault history (registers 601-699) |

Fast-tier data includes everything that changes in real-time during operation. Medium-tier data only changes on user action or fault events. Slow-tier data is historical and rarely changes.

## Fault Codes

The component decodes fault codes into human-readable descriptions:

| Code | Description |
| :--- | :--- |
| 1 | Input Error |
| 2 | High Pressure |
| 3 | Low Pressure |
| 4 | Freeze Detect FP2 |
| 5 | Freeze Detect FP1 |
| 7 | Condensate Overflow |
| 8 | Over/Under Voltage |
| 9 | AirF/RPM |
| 10 | Compressor Monitor |
| 11 | FP1/2 Sensor Error |
| 12 | RefPerfrm Error |
| 13 | Non-Critical AXB Sensor Error |
| 14 | Critical AXB Sensor Error |
| 15 | Hot Water Limit |
| 16 | VS Pump Error |
| 17 | Communicating Thermostat Error |
| 18 | Non-Critical Communications Error |
| 19 | Critical Communications Error |
| 21 | Low Loop Pressure |
| 22 | Communicating ECM Error |
| 41-61 | VS Drive Faults |
| 71-74 | EEV2 Faults |
| 99 | System Reset |

## Key Register Map

For reference, here are the key registers used by this component:

### System Registers
| Register | Description |
| :--- | :--- |
| 2 | ABC Version |
| 6 | Anti-Short-Cycle Countdown |
| 16 | Line Voltage |
| 19 | FP1 Temperature (Liquid Line) |
| 20 | FP2 Temperature (Air Coil) |
| 25 | Last Fault (bit 15 = lockout) |
| 30 | System Outputs (compressor, blower, aux, etc.) |
| 31 | System Status (LPS, HPS, emergency, load shed) |
| 88-91 | ABC Program Name (VS detection) |
| 92-103 | Model Number (12 registers, ASCII) |
| 105-109 | Serial Number (5 registers, ASCII) |
| 112 | Line Voltage Setting |
| 321-322 | VS Pump Min/Max Speed Settings |
| 340-342, 347 | ECM Blower Speed Settings |
| 404 | Blower Type (PSC/ECM/5-Speed) |
| 412 | Energy Monitor Level |
| 413 | Pump Type |
| 601-699 | Fault History (up to 10 faults) |
| 801 | AWL Thermostat Version |
| 807 | AWL AXB Version |
| 813 | AWL IZ2 Version |

### Thermostat Registers
| Register | Description |
| :--- | :--- |
| 502 | Ambient Temperature |
| 567 | Entering Air Temperature |
| 740 | Entering Air Temperature (AWL) |
| 741 | Relative Humidity |
| 742 | Outdoor Temperature |
| 745 | Heating Setpoint |
| 746 | Cooling Setpoint |
| 900 | Leaving Air Temperature |

### AXB Registers (Accessory Board)
| Register | Description |
| :--- | :--- |
| 400 | DHW Enabled |
| 401 | DHW Setpoint |
| 806 | AXB Installed Status |
| 1103 | AXB DIP Switch Settings |
| 1104 | AXB Outputs (DHW, loop pump, etc.) |
| 1109 | Heating Liquid Line Temperature |
| 1110 | Leaving Water Temperature |
| 1111 | Entering Water Temperature |
| 1114 | DHW Temperature |
| 1115 | Discharge Pressure |
| 1116 | Suction Pressure |
| 1117 | Water Flow |
| 1119 | Loop Pressure |

### VS Drive Registers (5/7-Series)
| Register | Description |
| :--- | :--- |
| 3000 | Compressor Desired Speed |
| 3001 | Compressor Actual Speed |
| 3322 | Discharge Pressure |
| 3323 | Suction Pressure |
| 3325 | Discharge Temperature |
| 3326 | Compressor Ambient Temperature |
| 3327 | Drive Temperature |
| 3422-3423 | VS Compressor Power (32-bit) |
| 3522 | Inverter Temperature |
| 3524 | Fan Speed |
| 3808 | EEV Open Percentage |
| 3903 | Suction Temperature |
| 3905 | Saturated Evaporator Discharge Temp |
| 3906 | Superheat Temperature |

### IZ2 Zone Registers
| Base Register | Offset | Description |
| :--- | :--- | :--- |
| 31007 | +3/zone | Zone Ambient Temperature |
| 31008 | +3/zone | Zone Config 1 (fan, cooling SP) |
| 31009 | +3/zone | Zone Config 2 (mode, heating SP) |
| 31200 | +3/zone | Zone Config 3 (priority, size) |
| 21202 | +9/zone | Zone Mode Write |
| 21203 | +9/zone | Zone Heating SP Write |
| 21204 | +9/zone | Zone Cooling SP Write |

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

## Community Projects

- [**Project Box for WaterFurnace Aurora**](https://github.com/benpeart/esphome_waterfurnace_aurora_projectbox) by [Ben Peart](https://github.com/benpeart) — Parts list, assembly instructions, and a 3D-printable project box to house the ESP8266 + MAX485 module.

## Credits

Huge thanks to [Cody Cutrer (ccutrer)](https://github.com/ccutrer) for reverse engineering the protocol and creating the original Ruby implementation.

## License

This project is licensed under the MIT License.
