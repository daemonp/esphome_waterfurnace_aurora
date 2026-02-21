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

# Binary sensor configuration keys
CONF_COMPRESSOR_RUNNING = "compressor_running"
CONF_BLOWER_RUNNING = "blower_running"
CONF_AUX_HEAT_RUNNING = "aux_heat_running"
CONF_DHW_RUNNING = "dhw_running"
CONF_LOOP_PUMP_RUNNING = "loop_pump_running"
CONF_LOCKOUT = "lockout"
CONF_HUMIDIFIER_RUNNING = "humidifier_running"
CONF_DEHUMIDIFIER_RUNNING = "dehumidifier_running"
CONF_LPS = "low_pressure_switch"
CONF_HPS = "high_pressure_switch"
CONF_EMERGENCY_SHUTDOWN = "emergency_shutdown"
CONF_LOAD_SHED = "load_shed"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        cv.Optional(CONF_COMPRESSOR_RUNNING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_RUNNING,
        ),
        cv.Optional(CONF_BLOWER_RUNNING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_RUNNING,
        ),
        cv.Optional(CONF_AUX_HEAT_RUNNING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_RUNNING,
        ),
        cv.Optional(CONF_DHW_RUNNING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_RUNNING,
        ),
        cv.Optional(CONF_LOOP_PUMP_RUNNING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_RUNNING,
        ),
        cv.Optional(CONF_LOCKOUT): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
        ),
        cv.Optional(CONF_HUMIDIFIER_RUNNING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_RUNNING,
        ),
        cv.Optional(CONF_DEHUMIDIFIER_RUNNING): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_RUNNING,
        ),
        cv.Optional(CONF_LPS): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
        ),
        cv.Optional(CONF_HPS): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
        ),
        cv.Optional(CONF_EMERGENCY_SHUTDOWN): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_PROBLEM,
        ),
        cv.Optional(CONF_LOAD_SHED): binary_sensor.binary_sensor_schema(
            device_class=DEVICE_CLASS_RUNNING,
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])

    if CONF_COMPRESSOR_RUNNING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_COMPRESSOR_RUNNING])
        cg.add(parent.set_compressor_binary_sensor(sens))

    if CONF_BLOWER_RUNNING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_BLOWER_RUNNING])
        cg.add(parent.set_blower_binary_sensor(sens))

    if CONF_AUX_HEAT_RUNNING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_AUX_HEAT_RUNNING])
        cg.add(parent.set_aux_heat_binary_sensor(sens))

    if CONF_DHW_RUNNING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_DHW_RUNNING])
        cg.add(parent.set_dhw_running_binary_sensor(sens))

    if CONF_LOOP_PUMP_RUNNING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_LOOP_PUMP_RUNNING])
        cg.add(parent.set_loop_pump_binary_sensor(sens))

    if CONF_LOCKOUT in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_LOCKOUT])
        cg.add(parent.set_lockout_binary_sensor(sens))

    if CONF_HUMIDIFIER_RUNNING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_HUMIDIFIER_RUNNING])
        cg.add(parent.set_humidifier_running_binary_sensor(sens))

    if CONF_DEHUMIDIFIER_RUNNING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_DEHUMIDIFIER_RUNNING])
        cg.add(parent.set_dehumidifier_running_binary_sensor(sens))

    if CONF_LPS in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_LPS])
        cg.add(parent.set_lps_binary_sensor(sens))

    if CONF_HPS in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_HPS])
        cg.add(parent.set_hps_binary_sensor(sens))

    if CONF_EMERGENCY_SHUTDOWN in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_EMERGENCY_SHUTDOWN])
        cg.add(parent.set_emergency_shutdown_binary_sensor(sens))

    if CONF_LOAD_SHED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_LOAD_SHED])
        cg.add(parent.set_load_shed_binary_sensor(sens))
