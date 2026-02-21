# Exposed Entities

Complete reference of all Home Assistant entities created by this component.

## Sensors

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

### Derived Sensor Details

**COP (Coefficient of Performance)** — Computed on-device each cycle:
- **Heating mode**: `Heat_of_Rejection / (Total_Watts * 3.412)`
- **Cooling mode**: `Heat_of_Extraction / (Total_Watts * 3.412)`
- Returns `0.0` when the compressor is off. Values are clamped to the 0.5-15.0 range; outliers are rejected as sensor noise.

**Subcool Temperature** — Automatically selects the correct register based on the current operating mode: register 1135 during heating, register 1136 during cooling (based on reversing valve state).

**Approach Temperature** — Heat exchanger approach:
- **Heating mode**: `Leaving Water - Saturated Condenser`
- **Cooling mode**: `Saturated Condenser - Entering Water`

## Binary Sensors

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

## Text Sensors

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

### `current_mode` Values

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

## Controls (Numbers)

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

## Switches

| Switch | Description |
| :--- | :--- |
| `dhw_enabled` | Enable/disable domestic hot water |

## Buttons

| Button | Description |
| :--- | :--- |
| `clear_fault_history` | Clear the fault history log |

## Climate Entities

The component creates climate entities for thermostat control:

- **Main thermostat** — Full HVAC control (heat, cool, auto, emergency heat) with dual setpoints and fan modes
- **IZ2 Zone thermostats** (zones 1-6) — Per-zone climate entities with independent temperature, setpoints, mode, and fan controls

> **Note**: On IZ2 systems, disable the main thermostat entity and use zone-specific entities instead. See [IZ2 Zone Configuration](../README.md#iz2-zone-configuration) in the README.
