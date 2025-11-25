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
- **VS Drive**: Compressor speed, drive/inverter temperatures (5/7-Series)
- **Energy**: Power consumption for compressor, blower, aux heat, loop pump
- **Loop**: Water flow rate, loop pressure
- **Humidity**: Relative humidity, humidification/dehumidification targets
- **System Status**: LPS/HPS switches, lockout, emergency shutdown, load shed

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
- Current operating mode (Heating, Cooling, Standby, etc.)
- Anti-short-cycle countdown

## Hardware Requirements

1. **ESP32 or ESP8266 Development Board** (e.g., Wemos D1 Mini, NodeMCU, ESP32-DevKit)
2. **TTL to RS485 Adapter** (MAX485 or similar 3.3V compatible module)
3. **RJ45 Connector/Cable**: To connect to the AID Tool port on the heat pump

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
  waterfurnace.aurora: github://damonp/esphome_waterfurnace_aurora/waterfurnace_aurora.yaml@master

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
| `update_interval` | `5s` | How often to poll the heat pump for data |

### IZ2 Zone Configuration

If you have an IntelliZone 2 system, uncomment the zone climate entries in the YAML or add them manually:

```yaml
climate:
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
  # ... up to zone 6
```

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
| `current_mode` | Current operating mode (Heating, Cooling, Standby, etc.) |
| `hvac_mode` | Configured HVAC mode (Off, Auto, Cool, Heat, E-Heat) |
| `fan_mode` | Configured fan mode (Auto, Continuous, Intermittent) |
| `fault_description` | Human-readable fault description |
| `model_number` | Heat pump model number (registers 92-103) |
| `serial_number` | Heat pump serial number (registers 105-109) |

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
| 92-103 | Model Number (12 registers, ASCII) |
| 105-109 | Serial Number (5 registers, ASCII) |
| 112 | Line Voltage Setting |

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
| 3327 | Drive Temperature |
| 3522 | Inverter Temperature |
| 3808 | EEV Open Percentage |
| 3903 | Suction Temperature |
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

### Missing Sensors
- Some sensors only appear if the required hardware is present (AXB, VS drive, IZ2)
- Check logs for "AXB detected", "VS Drive detected", "IZ2 detected"

## Credits

Huge thanks to [Cody Cutrer (ccutrer)](https://github.com/ccutrer) for reverse engineering the protocol and creating the original Ruby implementation.

## License

This project is licensed under the MIT License.
