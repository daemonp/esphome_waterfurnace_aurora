import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_PRESSURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
)

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

# Sensor configuration keys
CONF_ENTERING_AIR_TEMPERATURE = "entering_air_temperature"
CONF_LEAVING_AIR_TEMPERATURE = "leaving_air_temperature"
CONF_AMBIENT_TEMPERATURE = "ambient_temperature"
CONF_OUTDOOR_TEMPERATURE = "outdoor_temperature"
CONF_ENTERING_WATER_TEMPERATURE = "entering_water_temperature"
CONF_LEAVING_WATER_TEMPERATURE = "leaving_water_temperature"
CONF_HEATING_SETPOINT = "heating_setpoint"
CONF_COOLING_SETPOINT = "cooling_setpoint"
CONF_HUMIDITY = "humidity"
CONF_DHW_TEMPERATURE = "dhw_temperature"
CONF_DHW_SETPOINT = "dhw_setpoint"
CONF_COMPRESSOR_SPEED = "compressor_speed"
CONF_DISCHARGE_PRESSURE = "discharge_pressure"
CONF_SUCTION_PRESSURE = "suction_pressure"
CONF_EEV_OPEN_PERCENTAGE = "eev_open_percentage"
CONF_SUPERHEAT_TEMPERATURE = "superheat_temperature"
CONF_LINE_VOLTAGE = "line_voltage"
CONF_TOTAL_WATTS = "total_watts"
CONF_COMPRESSOR_WATTS = "compressor_watts"
CONF_BLOWER_WATTS = "blower_watts"
CONF_AUX_HEAT_WATTS = "aux_heat_watts"
CONF_PUMP_WATTS = "pump_watts"
CONF_WATERFLOW = "waterflow"
CONF_LOOP_PRESSURE = "loop_pressure"
CONF_FAULT_CODE = "fault_code"
CONF_LOCKOUT_FAULT_CODE = "lockout_fault_code"
CONF_FP1_TEMPERATURE = "fp1_temperature"
CONF_FP2_TEMPERATURE = "fp2_temperature"
CONF_LINE_VOLTAGE_SETTING = "line_voltage_setting"
CONF_ANTI_SHORT_CYCLE = "anti_short_cycle"
# Additional VS Drive sensors
CONF_COMPRESSOR_DESIRED_SPEED = "compressor_desired_speed"
CONF_DISCHARGE_TEMPERATURE = "discharge_temperature"
CONF_SUCTION_TEMPERATURE = "suction_temperature"
CONF_VS_DRIVE_TEMPERATURE = "vs_drive_temperature"
CONF_VS_INVERTER_TEMPERATURE = "vs_inverter_temperature"

# Blower/ECM sensors
CONF_BLOWER_SPEED = "blower_speed"
CONF_BLOWER_ONLY_SPEED = "blower_only_speed"
CONF_LO_COMPRESSOR_SPEED = "lo_compressor_speed"
CONF_HI_COMPRESSOR_SPEED = "hi_compressor_speed"
CONF_AUX_HEAT_SPEED = "aux_heat_speed"

# VS Pump sensors
CONF_PUMP_SPEED = "pump_speed"
CONF_PUMP_MIN_SPEED = "pump_min_speed"
CONF_PUMP_MAX_SPEED = "pump_max_speed"

# Refrigeration monitoring sensors
CONF_HEATING_LIQUID_LINE_TEMP = "heating_liquid_line_temperature"
CONF_SATURATED_CONDENSER_TEMP = "saturated_condenser_temperature"
CONF_SUBCOOL_TEMPERATURE = "subcool_temperature"
CONF_HEAT_OF_EXTRACTION = "heat_of_extraction"
CONF_HEAT_OF_REJECTION = "heat_of_rejection"

# Additional VS Drive sensors (Phase 5 parity)
CONF_VS_FAN_SPEED = "vs_fan_speed"
CONF_VS_AMBIENT_TEMPERATURE = "vs_ambient_temperature"
CONF_VS_COMPRESSOR_WATTS = "vs_compressor_watts"
CONF_SAT_EVAP_DISCHARGE_TEMPERATURE = "saturated_evaporator_discharge_temperature"
CONF_AUX_HEAT_STAGE = "aux_heat_stage"

# VS Drive additional diagnostics
CONF_VS_ENTERING_WATER_TEMPERATURE = "vs_entering_water_temperature"
CONF_VS_LINE_VOLTAGE = "vs_line_voltage"
CONF_VS_THERMO_POWER = "vs_thermo_power"
CONF_VS_SUPPLY_VOLTAGE = "vs_supply_voltage"
CONF_VS_UDC_VOLTAGE = "vs_udc_voltage"

# AXB current sensors
CONF_BLOWER_AMPS = "blower_amps"
CONF_AUX_AMPS = "aux_amps"
CONF_COMPRESSOR1_AMPS = "compressor_1_amps"
CONF_COMPRESSOR2_AMPS = "compressor_2_amps"

# Humidifier sensors
CONF_HUMIDIFICATION_TARGET = "humidification_target"
CONF_DEHUMIDIFICATION_TARGET = "dehumidification_target"

# IZ2 desired speed sensors (from compressor.rb / blower.rb)
CONF_IZ2_COMPRESSOR_SPEED = "iz2_compressor_speed"
CONF_IZ2_BLOWER_SPEED = "iz2_blower_speed"

# Derived sensors (computed on-device, beyond Ruby gem)
CONF_COP = "cop"
CONF_WATER_DELTA_T = "water_delta_t"
CONF_APPROACH_TEMPERATURE = "approach_temperature"

# Unit definitions
UNIT_FAHRENHEIT = "°F"
UNIT_PSI = "psi"
UNIT_GPM = "gpm"
UNIT_AMPERE = "A"

# Sensor schema for temperature sensors
TEMPERATURE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_FAHRENHEIT,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

# Sensor schema for pressure sensors
PRESSURE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_PSI,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_PRESSURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

# Sensor schema for power sensors
POWER_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_WATT,
    accuracy_decimals=0,
    device_class=DEVICE_CLASS_POWER,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        # Temperature sensors
        cv.Optional(CONF_ENTERING_AIR_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_LEAVING_AIR_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_AMBIENT_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_OUTDOOR_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_ENTERING_WATER_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_LEAVING_WATER_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_HEATING_SETPOINT): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_COOLING_SETPOINT): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_DHW_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_DHW_SETPOINT): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_SUPERHEAT_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        # Humidity
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Compressor
        cv.Optional(CONF_COMPRESSOR_SPEED): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Pressures
        cv.Optional(CONF_DISCHARGE_PRESSURE): PRESSURE_SENSOR_SCHEMA,
        cv.Optional(CONF_SUCTION_PRESSURE): PRESSURE_SENSOR_SCHEMA,
        cv.Optional(CONF_LOOP_PRESSURE): PRESSURE_SENSOR_SCHEMA,
        # EEV
        cv.Optional(CONF_EEV_OPEN_PERCENTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Voltage
        cv.Optional(CONF_LINE_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Power sensors
        cv.Optional(CONF_TOTAL_WATTS): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_COMPRESSOR_WATTS): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_BLOWER_WATTS): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_AUX_HEAT_WATTS): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_PUMP_WATTS): POWER_SENSOR_SCHEMA,
        # Flow
        cv.Optional(CONF_WATERFLOW): sensor.sensor_schema(
            unit_of_measurement=UNIT_GPM,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Fault
        cv.Optional(CONF_FAULT_CODE): sensor.sensor_schema(
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional(CONF_LOCKOUT_FAULT_CODE): sensor.sensor_schema(
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        # FP1/FP2 refrigerant temperatures
        cv.Optional(CONF_FP1_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_FP2_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        # Line voltage setting (configured voltage, not measured)
        cv.Optional(CONF_LINE_VOLTAGE_SETTING): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        # Anti-short-cycle countdown (seconds)
        cv.Optional(CONF_ANTI_SHORT_CYCLE): sensor.sensor_schema(
            unit_of_measurement="s",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        # Additional VS Drive sensors
        cv.Optional(CONF_COMPRESSOR_DESIRED_SPEED): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DISCHARGE_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_SUCTION_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_VS_DRIVE_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_VS_INVERTER_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        # Blower/ECM sensors
        cv.Optional(CONF_BLOWER_SPEED): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_BLOWER_ONLY_SPEED): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_LO_COMPRESSOR_SPEED): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_HI_COMPRESSOR_SPEED): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_AUX_HEAT_SPEED): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # VS Pump sensors
        cv.Optional(CONF_PUMP_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PUMP_MIN_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_PUMP_MAX_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Refrigeration monitoring sensors
        cv.Optional(CONF_HEATING_LIQUID_LINE_TEMP): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_SATURATED_CONDENSER_TEMP): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_SUBCOOL_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_HEAT_OF_EXTRACTION): sensor.sensor_schema(
            unit_of_measurement="BTU/h",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_HEAT_OF_REJECTION): sensor.sensor_schema(
            unit_of_measurement="BTU/h",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Additional VS Drive sensors (Phase 5 parity)
        cv.Optional(CONF_VS_FAN_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VS_AMBIENT_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_VS_COMPRESSOR_WATTS): POWER_SENSOR_SCHEMA,
        cv.Optional(CONF_SAT_EVAP_DISCHARGE_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_AUX_HEAT_STAGE): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # VS Drive additional diagnostics
        cv.Optional(CONF_VS_ENTERING_WATER_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_VS_LINE_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VS_THERMO_POWER): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VS_SUPPLY_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_VS_UDC_VOLTAGE): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # AXB current sensors
        cv.Optional(CONF_BLOWER_AMPS): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class="current",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_AUX_AMPS): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class="current",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_COMPRESSOR1_AMPS): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class="current",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_COMPRESSOR2_AMPS): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            accuracy_decimals=1,
            device_class="current",
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Humidifier sensors
        cv.Optional(CONF_HUMIDIFICATION_TARGET): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_DEHUMIDIFICATION_TARGET): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # IZ2 desired speed sensors
        cv.Optional(CONF_IZ2_COMPRESSOR_SPEED): sensor.sensor_schema(
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_IZ2_BLOWER_SPEED): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        # Derived sensors (computed on-device)
        cv.Optional(CONF_COP): sensor.sensor_schema(
            accuracy_decimals=2,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_WATER_DELTA_T): TEMPERATURE_SENSOR_SCHEMA,
        cv.Optional(CONF_APPROACH_TEMPERATURE): TEMPERATURE_SENSOR_SCHEMA,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])

    # Temperature sensors
    if CONF_ENTERING_AIR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_ENTERING_AIR_TEMPERATURE])
        cg.add(parent.set_entering_air_sensor(sens))

    if CONF_LEAVING_AIR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_LEAVING_AIR_TEMPERATURE])
        cg.add(parent.set_leaving_air_sensor(sens))

    if CONF_AMBIENT_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_AMBIENT_TEMPERATURE])
        cg.add(parent.set_ambient_temp_sensor(sens))

    if CONF_OUTDOOR_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_OUTDOOR_TEMPERATURE])
        cg.add(parent.set_outdoor_temp_sensor(sens))

    if CONF_ENTERING_WATER_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_ENTERING_WATER_TEMPERATURE])
        cg.add(parent.set_entering_water_sensor(sens))

    if CONF_LEAVING_WATER_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_LEAVING_WATER_TEMPERATURE])
        cg.add(parent.set_leaving_water_sensor(sens))

    if CONF_HEATING_SETPOINT in config:
        sens = await sensor.new_sensor(config[CONF_HEATING_SETPOINT])
        cg.add(parent.set_heating_setpoint_sensor(sens))

    if CONF_COOLING_SETPOINT in config:
        sens = await sensor.new_sensor(config[CONF_COOLING_SETPOINT])
        cg.add(parent.set_cooling_setpoint_sensor(sens))

    if CONF_DHW_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_DHW_TEMPERATURE])
        cg.add(parent.set_dhw_temp_sensor(sens))

    if CONF_DHW_SETPOINT in config:
        sens = await sensor.new_sensor(config[CONF_DHW_SETPOINT])
        cg.add(parent.set_dhw_setpoint_sensor(sens))

    if CONF_SUPERHEAT_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_SUPERHEAT_TEMPERATURE])
        cg.add(parent.set_superheat_sensor(sens))

    # Humidity
    if CONF_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(parent.set_humidity_sensor(sens))

    # Compressor
    if CONF_COMPRESSOR_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_SPEED])
        cg.add(parent.set_compressor_speed_sensor(sens))

    # Pressures
    if CONF_DISCHARGE_PRESSURE in config:
        sens = await sensor.new_sensor(config[CONF_DISCHARGE_PRESSURE])
        cg.add(parent.set_discharge_pressure_sensor(sens))

    if CONF_SUCTION_PRESSURE in config:
        sens = await sensor.new_sensor(config[CONF_SUCTION_PRESSURE])
        cg.add(parent.set_suction_pressure_sensor(sens))

    if CONF_LOOP_PRESSURE in config:
        sens = await sensor.new_sensor(config[CONF_LOOP_PRESSURE])
        cg.add(parent.set_loop_pressure_sensor(sens))

    # EEV
    if CONF_EEV_OPEN_PERCENTAGE in config:
        sens = await sensor.new_sensor(config[CONF_EEV_OPEN_PERCENTAGE])
        cg.add(parent.set_eev_open_sensor(sens))

    # Voltage
    if CONF_LINE_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_LINE_VOLTAGE])
        cg.add(parent.set_line_voltage_sensor(sens))

    # Power sensors
    if CONF_TOTAL_WATTS in config:
        sens = await sensor.new_sensor(config[CONF_TOTAL_WATTS])
        cg.add(parent.set_total_watts_sensor(sens))

    if CONF_COMPRESSOR_WATTS in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_WATTS])
        cg.add(parent.set_compressor_watts_sensor(sens))

    if CONF_BLOWER_WATTS in config:
        sens = await sensor.new_sensor(config[CONF_BLOWER_WATTS])
        cg.add(parent.set_blower_watts_sensor(sens))

    if CONF_AUX_HEAT_WATTS in config:
        sens = await sensor.new_sensor(config[CONF_AUX_HEAT_WATTS])
        cg.add(parent.set_aux_watts_sensor(sens))

    if CONF_PUMP_WATTS in config:
        sens = await sensor.new_sensor(config[CONF_PUMP_WATTS])
        cg.add(parent.set_pump_watts_sensor(sens))

    # Flow
    if CONF_WATERFLOW in config:
        sens = await sensor.new_sensor(config[CONF_WATERFLOW])
        cg.add(parent.set_waterflow_sensor(sens))

    # Fault
    if CONF_FAULT_CODE in config:
        sens = await sensor.new_sensor(config[CONF_FAULT_CODE])
        cg.add(parent.set_fault_code_sensor(sens))

    if CONF_LOCKOUT_FAULT_CODE in config:
        sens = await sensor.new_sensor(config[CONF_LOCKOUT_FAULT_CODE])
        cg.add(parent.set_lockout_fault_sensor(sens))

    # FP1/FP2 refrigerant temperatures
    if CONF_FP1_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_FP1_TEMPERATURE])
        cg.add(parent.set_fp1_sensor(sens))

    if CONF_FP2_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_FP2_TEMPERATURE])
        cg.add(parent.set_fp2_sensor(sens))

    # Line voltage setting
    if CONF_LINE_VOLTAGE_SETTING in config:
        sens = await sensor.new_sensor(config[CONF_LINE_VOLTAGE_SETTING])
        cg.add(parent.set_line_voltage_setting_sensor(sens))

    # Anti-short-cycle countdown
    if CONF_ANTI_SHORT_CYCLE in config:
        sens = await sensor.new_sensor(config[CONF_ANTI_SHORT_CYCLE])
        cg.add(parent.set_anti_short_cycle_sensor(sens))

    # Additional VS Drive sensors
    if CONF_COMPRESSOR_DESIRED_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR_DESIRED_SPEED])
        cg.add(parent.set_compressor_desired_speed_sensor(sens))

    if CONF_DISCHARGE_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_DISCHARGE_TEMPERATURE])
        cg.add(parent.set_discharge_temp_sensor(sens))

    if CONF_SUCTION_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_SUCTION_TEMPERATURE])
        cg.add(parent.set_suction_temp_sensor(sens))

    if CONF_VS_DRIVE_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_VS_DRIVE_TEMPERATURE])
        cg.add(parent.set_vs_drive_temp_sensor(sens))

    if CONF_VS_INVERTER_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_VS_INVERTER_TEMPERATURE])
        cg.add(parent.set_vs_inverter_temp_sensor(sens))

    # Blower/ECM sensors
    if CONF_BLOWER_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_BLOWER_SPEED])
        cg.add(parent.set_blower_speed_sensor(sens))

    if CONF_BLOWER_ONLY_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_BLOWER_ONLY_SPEED])
        cg.add(parent.set_blower_only_speed_sensor(sens))

    if CONF_LO_COMPRESSOR_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_LO_COMPRESSOR_SPEED])
        cg.add(parent.set_lo_compressor_speed_sensor(sens))

    if CONF_HI_COMPRESSOR_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_HI_COMPRESSOR_SPEED])
        cg.add(parent.set_hi_compressor_speed_sensor(sens))

    if CONF_AUX_HEAT_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_AUX_HEAT_SPEED])
        cg.add(parent.set_aux_heat_speed_sensor(sens))

    # VS Pump sensors
    if CONF_PUMP_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_PUMP_SPEED])
        cg.add(parent.set_pump_speed_sensor(sens))

    if CONF_PUMP_MIN_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_PUMP_MIN_SPEED])
        cg.add(parent.set_pump_min_speed_sensor(sens))

    if CONF_PUMP_MAX_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_PUMP_MAX_SPEED])
        cg.add(parent.set_pump_max_speed_sensor(sens))

    # Refrigeration monitoring sensors
    if CONF_HEATING_LIQUID_LINE_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_HEATING_LIQUID_LINE_TEMP])
        cg.add(parent.set_heating_liquid_line_temp_sensor(sens))

    if CONF_SATURATED_CONDENSER_TEMP in config:
        sens = await sensor.new_sensor(config[CONF_SATURATED_CONDENSER_TEMP])
        cg.add(parent.set_saturated_condenser_temp_sensor(sens))

    if CONF_SUBCOOL_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_SUBCOOL_TEMPERATURE])
        cg.add(parent.set_subcool_temp_sensor(sens))

    if CONF_HEAT_OF_EXTRACTION in config:
        sens = await sensor.new_sensor(config[CONF_HEAT_OF_EXTRACTION])
        cg.add(parent.set_heat_of_extraction_sensor(sens))

    if CONF_HEAT_OF_REJECTION in config:
        sens = await sensor.new_sensor(config[CONF_HEAT_OF_REJECTION])
        cg.add(parent.set_heat_of_rejection_sensor(sens))

    # Additional VS Drive sensors (Phase 5 parity)
    if CONF_VS_FAN_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_VS_FAN_SPEED])
        cg.add(parent.set_vs_fan_speed_sensor(sens))

    if CONF_VS_AMBIENT_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_VS_AMBIENT_TEMPERATURE])
        cg.add(parent.set_vs_ambient_temp_sensor(sens))

    if CONF_VS_COMPRESSOR_WATTS in config:
        sens = await sensor.new_sensor(config[CONF_VS_COMPRESSOR_WATTS])
        cg.add(parent.set_vs_compressor_watts_sensor(sens))

    if CONF_SAT_EVAP_DISCHARGE_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_SAT_EVAP_DISCHARGE_TEMPERATURE])
        cg.add(parent.set_sat_evap_discharge_temp_sensor(sens))

    if CONF_AUX_HEAT_STAGE in config:
        sens = await sensor.new_sensor(config[CONF_AUX_HEAT_STAGE])
        cg.add(parent.set_aux_heat_stage_sensor(sens))

    # VS Drive additional diagnostics
    if CONF_VS_ENTERING_WATER_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_VS_ENTERING_WATER_TEMPERATURE])
        cg.add(parent.set_vs_entering_water_temp_sensor(sens))

    if CONF_VS_LINE_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_VS_LINE_VOLTAGE])
        cg.add(parent.set_vs_line_voltage_sensor(sens))

    if CONF_VS_THERMO_POWER in config:
        sens = await sensor.new_sensor(config[CONF_VS_THERMO_POWER])
        cg.add(parent.set_vs_thermo_power_sensor(sens))

    if CONF_VS_SUPPLY_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_VS_SUPPLY_VOLTAGE])
        cg.add(parent.set_vs_supply_voltage_sensor(sens))

    if CONF_VS_UDC_VOLTAGE in config:
        sens = await sensor.new_sensor(config[CONF_VS_UDC_VOLTAGE])
        cg.add(parent.set_vs_udc_voltage_sensor(sens))

    # AXB current sensors
    if CONF_BLOWER_AMPS in config:
        sens = await sensor.new_sensor(config[CONF_BLOWER_AMPS])
        cg.add(parent.set_blower_amps_sensor(sens))

    if CONF_AUX_AMPS in config:
        sens = await sensor.new_sensor(config[CONF_AUX_AMPS])
        cg.add(parent.set_aux_amps_sensor(sens))

    if CONF_COMPRESSOR1_AMPS in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR1_AMPS])
        cg.add(parent.set_compressor1_amps_sensor(sens))

    if CONF_COMPRESSOR2_AMPS in config:
        sens = await sensor.new_sensor(config[CONF_COMPRESSOR2_AMPS])
        cg.add(parent.set_compressor2_amps_sensor(sens))

    # Humidifier sensors
    if CONF_HUMIDIFICATION_TARGET in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDIFICATION_TARGET])
        cg.add(parent.set_humidification_target_sensor(sens))

    if CONF_DEHUMIDIFICATION_TARGET in config:
        sens = await sensor.new_sensor(config[CONF_DEHUMIDIFICATION_TARGET])
        cg.add(parent.set_dehumidification_target_sensor(sens))

    # IZ2 desired speed sensors
    if CONF_IZ2_COMPRESSOR_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_IZ2_COMPRESSOR_SPEED])
        cg.add(parent.set_iz2_compressor_speed_sensor(sens))

    if CONF_IZ2_BLOWER_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_IZ2_BLOWER_SPEED])
        cg.add(parent.set_iz2_blower_speed_sensor(sens))

    # Derived sensors (computed on-device)
    if CONF_COP in config:
        sens = await sensor.new_sensor(config[CONF_COP])
        cg.add(parent.set_cop_sensor(sens))

    if CONF_WATER_DELTA_T in config:
        sens = await sensor.new_sensor(config[CONF_WATER_DELTA_T])
        cg.add(parent.set_water_delta_t_sensor(sens))

    if CONF_APPROACH_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_APPROACH_TEMPERATURE])
        cg.add(parent.set_approach_temp_sensor(sens))
