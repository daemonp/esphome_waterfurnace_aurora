import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_PRESSURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
)

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID, UNIT_FAHRENHEIT

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

# Unit definitions
UNIT_PSI = "psi"
UNIT_GPM = "gpm"

# Shared sensor schemas
TEMPERATURE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_FAHRENHEIT,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

PRESSURE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_PSI,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_PRESSURE,
    state_class=STATE_CLASS_MEASUREMENT,
)

POWER_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_WATT,
    accuracy_decimals=0,
    device_class=DEVICE_CLASS_POWER,
    state_class=STATE_CLASS_MEASUREMENT,
)

VOLTAGE_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_VOLT,
    accuracy_decimals=0,
    device_class=DEVICE_CLASS_VOLTAGE,
    state_class=STATE_CLASS_MEASUREMENT,
)

CURRENT_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_AMPERE,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_CURRENT,
    state_class=STATE_CLASS_MEASUREMENT,
)

SPEED_SENSOR_SCHEMA = sensor.sensor_schema(
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)

PERCENT_SENSOR_SCHEMA = sensor.sensor_schema(
    unit_of_measurement=UNIT_PERCENT,
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)

# Single dictionary driving both CONFIG_SCHEMA and to_code().
# Key = YAML config key (also used to derive C++ setter via f"set_{key}_sensor").
SENSORS = {
    # Temperature sensors
    "entering_air_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "leaving_air_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "ambient_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "outdoor_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "entering_water_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "leaving_water_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "heating_setpoint": TEMPERATURE_SENSOR_SCHEMA,
    "cooling_setpoint": TEMPERATURE_SENSOR_SCHEMA,
    "dhw_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "dhw_setpoint": TEMPERATURE_SENSOR_SCHEMA,
    "superheat_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "fp1_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "fp2_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "discharge_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "suction_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "vs_drive_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "vs_inverter_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "heating_liquid_line_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "saturated_condenser_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "subcool_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "vs_ambient_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "saturated_evaporator_discharge_temperature": TEMPERATURE_SENSOR_SCHEMA,
    "vs_entering_water_temperature": TEMPERATURE_SENSOR_SCHEMA,
    # Derived temperature sensors
    "water_delta_t": TEMPERATURE_SENSOR_SCHEMA,
    "approach_temperature": TEMPERATURE_SENSOR_SCHEMA,
    # Humidity
    "humidity": sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_HUMIDITY,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    # Compressor
    "compressor_speed": SPEED_SENSOR_SCHEMA,
    "compressor_desired_speed": SPEED_SENSOR_SCHEMA,
    # Pressures
    "discharge_pressure": PRESSURE_SENSOR_SCHEMA,
    "suction_pressure": PRESSURE_SENSOR_SCHEMA,
    "loop_pressure": PRESSURE_SENSOR_SCHEMA,
    # EEV
    "eev_open_percentage": sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    # Voltage
    "line_voltage": VOLTAGE_SENSOR_SCHEMA,
    "vs_line_voltage": VOLTAGE_SENSOR_SCHEMA,
    "vs_supply_voltage": VOLTAGE_SENSOR_SCHEMA,
    "vs_udc_voltage": VOLTAGE_SENSOR_SCHEMA,
    # Power sensors
    "total_watts": POWER_SENSOR_SCHEMA,
    "compressor_watts": POWER_SENSOR_SCHEMA,
    "blower_watts": POWER_SENSOR_SCHEMA,
    "aux_heat_watts": POWER_SENSOR_SCHEMA,
    "pump_watts": POWER_SENSOR_SCHEMA,
    "vs_compressor_watts": POWER_SENSOR_SCHEMA,
    # Flow
    "waterflow": sensor.sensor_schema(
        unit_of_measurement=UNIT_GPM,
        accuracy_decimals=1,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    # Fault
    "fault_code": sensor.sensor_schema(
        accuracy_decimals=0,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    "lockout_fault_code": sensor.sensor_schema(
        accuracy_decimals=0,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    # Line voltage setting (configured voltage, not measured)
    "line_voltage_setting": sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_VOLTAGE,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    # Anti-short-cycle countdown (seconds)
    "anti_short_cycle": sensor.sensor_schema(
        unit_of_measurement="s",
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    # Blower/ECM sensors
    "blower_speed": SPEED_SENSOR_SCHEMA,
    "blower_only_speed": SPEED_SENSOR_SCHEMA,
    "lo_compressor_speed": SPEED_SENSOR_SCHEMA,
    "hi_compressor_speed": SPEED_SENSOR_SCHEMA,
    "aux_heat_speed": SPEED_SENSOR_SCHEMA,
    # VS Pump sensors
    "pump_speed": PERCENT_SENSOR_SCHEMA,
    "pump_min_speed": PERCENT_SENSOR_SCHEMA,
    "pump_max_speed": PERCENT_SENSOR_SCHEMA,
    # Heat transfer
    "heat_of_extraction": sensor.sensor_schema(
        unit_of_measurement="BTU/h",
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    "heat_of_rejection": sensor.sensor_schema(
        unit_of_measurement="BTU/h",
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    # Additional VS Drive sensors
    "vs_fan_speed": PERCENT_SENSOR_SCHEMA,
    "aux_heat_stage": SPEED_SENSOR_SCHEMA,
    "vs_thermo_power": PERCENT_SENSOR_SCHEMA,
    # AXB current sensors
    "blower_amps": CURRENT_SENSOR_SCHEMA,
    "aux_amps": CURRENT_SENSOR_SCHEMA,
    "compressor_1_amps": CURRENT_SENSOR_SCHEMA,
    "compressor_2_amps": CURRENT_SENSOR_SCHEMA,
    # Humidifier sensors
    "humidification_target": sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_HUMIDITY,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    "dehumidification_target": sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT,
        accuracy_decimals=0,
        device_class=DEVICE_CLASS_HUMIDITY,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
    # IZ2 desired speed sensors
    "iz2_compressor_speed": SPEED_SENSOR_SCHEMA,
    "iz2_blower_speed": PERCENT_SENSOR_SCHEMA,
    # IZ2 demand sensors
    "iz2_fan_demand": sensor.sensor_schema(
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    "iz2_unit_demand": sensor.sensor_schema(
        accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    ),
    # Derived sensors (computed on-device)
    "cop": sensor.sensor_schema(
        accuracy_decimals=2,
        state_class=STATE_CLASS_MEASUREMENT,
    ),
}

CONFIG_SCHEMA = cv.Schema(
    {cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora)}
).extend(
    {cv.Optional(key): schema for key, schema in SENSORS.items()}
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])
    for key in SENSORS:
        if conf := config.get(key):
            sens = await sensor.new_sensor(conf)
            cg.add(getattr(parent, f"set_{key}_sensor")(sens))
