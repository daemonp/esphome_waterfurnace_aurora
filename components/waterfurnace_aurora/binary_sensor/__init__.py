import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    DEVICE_CLASS_RUNNING,
    DEVICE_CLASS_PROBLEM,
)

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

# Single dictionary driving both CONFIG_SCHEMA and to_code().
# Key = YAML config key (also used to derive C++ setter via f"set_{key}_binary_sensor").
BINARY_SENSORS = {
    "compressor_running": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING,
    ),
    "blower_running": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING,
    ),
    "aux_heat_running": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING,
    ),
    "dhw_running": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING,
    ),
    "loop_pump_running": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING,
    ),
    "lockout": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PROBLEM,
    ),
    "humidifier_running": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING,
    ),
    "dehumidifier_running": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING,
    ),
    "low_pressure_switch": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PROBLEM,
    ),
    "high_pressure_switch": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PROBLEM,
    ),
    "emergency_shutdown": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PROBLEM,
    ),
    "load_shed": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING,
    ),
    "fan_call": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING,
    ),
    "derated": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PROBLEM,
    ),
    "safe_mode": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_PROBLEM,
    ),
    "diverting_valve": binary_sensor.binary_sensor_schema(
        device_class=DEVICE_CLASS_RUNNING,
    ),
}

CONFIG_SCHEMA = cv.Schema(
    {cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora)}
).extend(
    {cv.Optional(key): schema for key, schema in BINARY_SENSORS.items()}
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])
    for key in BINARY_SENSORS:
        if conf := config.get(key):
            sens = await binary_sensor.new_binary_sensor(conf)
            cg.add(getattr(parent, f"set_{key}_binary_sensor")(sens))
