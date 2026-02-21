# Protocol & Register Reference

## Protocol Details

This component implements a custom Modbus RTU client directly on UART. Standard Modbus libraries cannot efficiently communicate with the Aurora because WaterFurnace uses proprietary function codes alongside standard ones.

### Function Codes

| Function Code | Name | Description |
| :--- | :--- | :--- |
| `0x03` | Read Holding Registers | Standard Modbus read (used for model/serial, fault history) |
| `0x06` | Write Single Register | Standard Modbus write (all write operations) |
| `0x41` (`'A'`) | Read Ranges | Proprietary: read multiple contiguous register ranges in one request |
| `0x42` (`'B'`) | Read Specific | Proprietary: read arbitrary non-sequential registers in one request |

Function code `0x42` is the primary read method. It sends a list of individual register addresses and receives their values in the same order, allowing 50-100+ scattered registers to be fetched in a single transaction.

### Why Not ESPHome's Built-in Modbus Controller?

See the [README](../README.md#why-not-esphomes-built-in-modbus-controller) for a summary of why this component implements its own Modbus stack instead of using the `modbus_controller` platform.

### Communication Settings

| Parameter | Value |
| :--- | :--- |
| **Baud Rate** | 19200 |
| **Data Bits** | 8 |
| **Parity** | Even |
| **Stop Bits** | 1 |
| **Default Address** | 1 |
| **CRC** | Standard Modbus CRC-16 (polynomial 0xA001, init 0xFFFF) |

### Read Retry Logic

The component implements automatic retry logic (default 2 retries, configurable 0-10) for communication failures. Retries include a 50 ms delay between attempts. Write operations are single-attempt only.

### Adaptive Polling

To reduce RS-485 bus load, the component uses three-tier adaptive polling. The `update_interval` setting controls the base (fast) tier; the other tiers are multiples of it.

| Tier | Default Interval | Data Polled |
| :--- | :--- | :--- |
| **Fast** | Every cycle (~5s) | Temperatures, compressor speed, power consumption, VS drive telemetry, water flow, blower/pump speed, IZ2 zone data |
| **Medium** | Every 6th cycle (~30s) | Setpoints, mode config, DHW settings, line voltage, ECM/pump speed settings, VS drive status flags (derate/safe/alarm), humidistat settings, AXB inputs, loop pressure |
| **Slow** | Every 60th cycle (~5 min) | Fault history (registers 601-699) |

Fast-tier data includes everything that changes in real-time during operation. Medium-tier data only changes on user action or fault events. Slow-tier data is historical and rarely changes.

## Register Map

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
| 214 | VS Drive Derate Flags |
| 216 | VS Drive Safe Mode Flags |
| 217-218 | VS Drive Alarm Codes |
| 321-322 | VS Pump Min/Max Speed Settings |
| 325 | VS Pump Speed (current) |
| 340-342, 347 | ECM Blower Speed Settings |
| 344 | Current ECM/Blower Speed |
| 362 | Active Dehumidify Status |
| 400 | DHW Enabled |
| 401 | DHW Setpoint |
| 404 | Blower Type (PSC/ECM/5-Speed) |
| 412 | Energy Monitor Level |
| 413 | Pump Type |
| 483 | IZ2 Zone Count |
| 601-699 | Fault History (up to 10 faults) |
| 800-801 | Thermostat Installed / Version |
| 806-807 | AXB Installed / Version |
| 812-813 | IZ2 Installed / Version |

### Thermostat Registers

| Register | Description |
| :--- | :--- |
| 502 | Ambient Temperature |
| 567 | Entering Air Temperature |
| 740 | Entering Air Temperature (AWL AXB) |
| 741 | Relative Humidity |
| 742 | Outdoor Temperature |
| 745 | Heating Setpoint |
| 746 | Cooling Setpoint |
| 900 | Leaving Air Temperature |
| 12005 | Fan Configuration (bits: continuous, intermittent) |
| 12006 | Heating Mode (bits 10:8) |
| 12309 | Humidistat Settings (non-IZ2) |
| 12310 | Humidistat Targets (non-IZ2) |

### AXB Registers (Accessory Board)

| Register | Description |
| :--- | :--- |
| 806 | AXB Installed Status |
| 1103 | AXB DIP Switch Settings |
| 1104 | AXB Outputs (DHW, loop pump, diverting valve) |
| 1109 | Heating Liquid Line Temperature |
| 1110 | Leaving Water Temperature |
| 1111 | Entering Water Temperature |
| 1114 | DHW Temperature |
| 1115 | Discharge Pressure |
| 1116 | Suction Pressure |
| 1117 | Water Flow |
| 1119 | Loop Pressure |
| 1134 | Saturated Condenser Temperature |
| 1135 | Subcool Heating |
| 1136 | Subcool Cooling |

### Energy Registers

| Register | Description |
| :--- | :--- |
| 1146-1147 | Compressor Watts (32-bit) |
| 1148-1149 | Blower Watts (32-bit) |
| 1150-1151 | Aux Heat Watts (32-bit) |
| 1152-1153 | Total Watts (32-bit) |
| 1154-1155 | Heat of Extraction (32-bit, signed) |
| 1156-1157 | Heat of Rejection (32-bit, signed) |
| 1164-1165 | Pump Watts (32-bit) |

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
| 3905 | Saturated Evaporator Discharge Temperature |
| 3906 | Superheat Temperature |

### IZ2 Zone Registers (Read)

| Base Register | Offset | Description |
| :--- | :--- | :--- |
| 31003 | - | IZ2 Outdoor Temperature |
| 31005 | - | IZ2 Demand |
| 31007 | +3/zone | Zone Ambient Temperature |
| 31008 | +3/zone | Zone Config 1 (fan, cooling SP) |
| 31009 | +3/zone | Zone Config 2 (mode, heating SP) |
| 31109 | - | IZ2 Humidistat Mode |
| 31110 | - | IZ2 Humidistat Targets |
| 31200 | +3/zone | Zone Config 3 (priority, size) |

### IZ2 Zone Registers (Write)

| Base Register | Offset | Description |
| :--- | :--- | :--- |
| 21114 | - | IZ2 Humidistat Settings |
| 21115 | - | IZ2 Humidistat Targets |
| 21202 | +9/zone | Zone Mode |
| 21203 | +9/zone | Zone Heating Setpoint |
| 21204 | +9/zone | Zone Cooling Setpoint |
| 21205 | +9/zone | Zone Fan Mode |
| 21206 | +9/zone | Zone Fan Intermittent On |
| 21207 | +9/zone | Zone Fan Intermittent Off |

### Thermostat Write Registers

| Register | Description |
| :--- | :--- |
| 12606 | HVAC Mode |
| 12619 | Heating Setpoint |
| 12620 | Cooling Setpoint |
| 12621 | Fan Mode |
| 12622 | Fan Intermittent On Time |
| 12623 | Fan Intermittent Off Time |

> **Note**: Several parameters use different registers for reading vs. writing (e.g., heating setpoint is read from 745 but written to 12619). This is a WaterFurnace protocol design choice.

## Fault Codes

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
