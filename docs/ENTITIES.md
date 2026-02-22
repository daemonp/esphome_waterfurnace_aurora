# Exposed Entities

Complete reference of all Home Assistant entities created by this component.

## Sensors

| Sensor | Unit | Description | Register |
| :--- | :--- | :--- | :--- |
| `entering_air_temperature` | °F | Return air temperature (air entering the unit) | 567/740 |
| `leaving_air_temperature` | °F | Supply air temperature (air leaving the unit) | 900 |
| `ambient_temperature` | °F | Room/zone temperature | 502 |
| `outdoor_temperature` | °F | Outdoor temperature (AWL) | 742 |
| `entering_water_temperature` | °F | Loop water entering the unit | 1111 |
| `leaving_water_temperature` | °F | Loop water leaving the unit | 1110 |
| `heating_setpoint` | °F | Current heating setpoint | 745 |
| `cooling_setpoint` | °F | Current cooling setpoint | 746 |
| `humidity` | % | Humidity level | 741 |
| `compressor_speed` | RPM | Actual compressor speed (VS) | 3001 |
| `compressor_desired_speed` | RPM | Target compressor speed (VS) | 3000 |
| `discharge_pressure` | psi | Refrigerant discharge pressure | 3322 |
| `suction_pressure` | psi | Refrigerant suction pressure | 3323 |
| `superheat_temperature` | °F | Superheat temperature | 3906 |
| `eev_open_percentage` | % | Electronic expansion valve opening | 3808 |
| `line_voltage` | V | Incoming line voltage | 16 |
| `total_watts` | W | Total unit power consumption | 1152-1153 |
| `compressor_watts` | W | Compressor power consumption | 1146-1147 |
| `blower_watts` | W | Indoor blower power consumption | 1148-1149 |
| `aux_heat_watts` | W | Aux electric heat power consumption | 1150-1151 |
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
| `blower_speed` | - | Current indoor blower ECM speed | 344 |
| `pump_speed` | % | Current loop pump speed | 325 |
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
| `aux_heat_stage` | - | Aux electric heat stage (0=off, 1=EH1, 2=EH1+EH2) | Derived (reg 30) |
| `blower_only_speed` | - | Indoor blower-only ECM speed setting | 340 |
| `lo_compressor_speed` | - | Low compressor ECM speed setting | 341 |
| `hi_compressor_speed` | - | High compressor ECM speed setting | 342 |
| `aux_heat_speed` | - | Aux electric heat ECM speed setting | 347 |
| `pump_min_speed` | % | Loop pump minimum speed setting | 321 |
| `pump_max_speed` | % | Loop pump maximum speed setting | 322 |
| `humidification_target` | % | Humidification setpoint | 12310/31110 |
| `dehumidification_target` | % | Dehumidification setpoint | 12310/31110 |
| `line_voltage_setting` | V | Configured line voltage setting | 112 |
| `vs_entering_water_temperature` | °F | VS Drive entering water temperature | 3330 |
| `vs_line_voltage` | V | VS Drive line voltage | 3331 |
| `vs_thermo_power` | % | VS Drive thermo power | 3332 |
| `vs_supply_voltage` | V | VS Drive supply voltage | 3424-3425 |
| `vs_udc_voltage` | V | VS Drive UDC voltage | 3523 |
| `blower_amps` | A | AXB blower current draw | 1105 |
| `aux_amps` | A | AXB aux heat current draw | 1106 |
| `compressor_1_amps` | A | AXB compressor 1 current draw | 1107 |
| `compressor_2_amps` | A | AXB compressor 2 current draw | 1108 |

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
| `blower_running` | Indoor blower is running | Register 30 (BLOWER bit) |
| `aux_heat_running` | Aux electric heat is active | Register 30 (EH1/EH2 bits) |
| `dhw_running` | DHW mode is active | Register 1104 (DHW bit) |
| `loop_pump_running` | Loop pump is running | Register 1104 (LOOP_PUMP bit) |
| `lockout` | System is in lockout | Register 25 (bit 15) |
| `low_pressure_switch` | LPS has tripped | Register 31 (bit 7) |
| `high_pressure_switch` | HPS has tripped | Register 31 (bit 8) |
| `emergency_shutdown` | Emergency shutdown active | Register 31 (bit 6) |
| `load_shed` | Load shed/demand response active | Register 31 (bit 9) |
| `fan_call` | Fan call (G signal from thermostat bus) | Register 31 (bit 4) |
| `derated` | Compressor is derated (fault 41-46) | Derived (register 25) |
| `safe_mode` | VS drive in safe mode (fault 47-49/72/74) | Derived (register 25) |
| `diverting_valve` | AXB diverting valve active | Register 1104 (bit 2) |
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
| `Fan Only` | Indoor blower running, no compressor |
| `Waiting` | Anti-short-cycle countdown active |
| `Lockout` | System locked out due to fault |

## Controls (Numbers)

| Control | Range | Description |
| :--- | :--- | :--- |
| `dhw_setpoint` | 100-140°F | DHW temperature setpoint |
| `blower_only_speed` | 1-12 | Indoor blower ECM speed for fan-only mode |
| `lo_compressor_speed` | 1-12 | Indoor blower ECM speed for low compressor |
| `hi_compressor_speed` | 1-12 | Indoor blower ECM speed for high compressor |
| `aux_heat_speed` | 1-12 | Indoor blower ECM speed for aux electric heat |
| `pump_min_speed` | 1-100% | Loop pump minimum speed |
| `pump_max_speed` | 1-100% | Loop pump maximum speed |
| `fan_intermittent_on` | 0-25 min | Fan on time (intermittent mode) |
| `fan_intermittent_off` | 5-40 min | Fan off time (intermittent mode) |
| `humidification_target` | 15-50% | Humidification setpoint (Symphony: Hum Setpoint) |
| `dehumidification_target` | 35-65% | Dehumidification setpoint (Symphony: Hum Setpoint) |

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

## Water Heater Entity

The component creates a water heater entity for DHW (Domestic Hot Water) control:

- **Domestic Hot Water** — Current water temperature, target setpoint (100-140°F), and mode (Off / Heat Pump)

## Humidistat (Humidifier / Dehumidifier Cards)

The Aurora has a built-in humidistat with **two independent targets**: a humidification target (15-50%) for adding moisture and a dehumidification target (35-65%) for removing moisture. Both are always active regardless of heating/cooling mode.

ESPHome does not have a native `humidifier` platform, so this component exposes humidistat controls as individual entities:

| Entity | Platform | Purpose |
| :--- | :--- | :--- |
| `humidity` | sensor | Current relative humidity (%) |
| `humidification_setpoint` | number | Humidification target (15-50%, writable) |
| `dehumidification_setpoint` | number | Dehumidification target (35-65%, writable) |
| `humidifier_mode` | select | Humidifier mode (Auto/Manual, writable) |
| `dehumidifier_mode` | select | Dehumidifier mode (Auto/Manual, writable) |
| `humidifier_running` | binary_sensor | Humidifier relay active |
| `dehumidifier_running` | binary_sensor | Active dehumidification or AXB relay active |

### Getting Proper Humidifier Cards in Lovelace

To display these as full humidifier/dehumidifier cards (with target slider, mode selector, and on/off toggle), create **Home Assistant template humidifier entities** that wrap the ESPHome entities above.

A ready-to-paste configuration is provided in **[`docs/ha_humidifier_templates.yaml`](ha_humidifier_templates.yaml)**. Copy it into your HA `configuration.yaml` and replace `waterfurnace_aurora` with your ESPHome device name.

This creates two entities:
- `humidifier.aurora_humidifier` — proper humidifier card (device_class: humidifier)
- `humidifier.aurora_dehumidifier` — proper dehumidifier card (device_class: dehumidifier)
