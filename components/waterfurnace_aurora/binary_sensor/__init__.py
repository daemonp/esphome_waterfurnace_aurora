import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import (
    DEVICE_CLASS_RUNNING,
    DEVICE_CLASS_PROBLEM,
)

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@damonmaria"]

# Binary sensor configuration keys
CONF_COMPRESSOR_RUNNING = "compressor_running"
CONF_BLOWER_RUNNING = "blower_running"
CONF_AUX_HEAT_RUNNING = "aux_heat_running"
CONF_DHW_RUNNING = "dhw_running"
CONF_LOOP_PUMP_RUNNING = "loop_pump_running"
CONF_LOCKOUT = "lockout"

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
