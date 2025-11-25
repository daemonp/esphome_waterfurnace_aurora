import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@damonmaria"]

# Text sensor configuration keys
CONF_CURRENT_MODE = "current_mode"
CONF_HVAC_MODE = "hvac_mode"
CONF_FAN_MODE = "fan_mode"
CONF_FAULT_DESCRIPTION = "fault_description"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        cv.Optional(CONF_CURRENT_MODE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_HVAC_MODE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_FAN_MODE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_FAULT_DESCRIPTION): text_sensor.text_sensor_schema(),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])

    if CONF_CURRENT_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_CURRENT_MODE])
        cg.add(parent.set_current_mode_sensor(sens))

    if CONF_HVAC_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_HVAC_MODE])
        cg.add(parent.set_hvac_mode_sensor(sens))

    if CONF_FAN_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_FAN_MODE])
        cg.add(parent.set_fan_mode_sensor(sens))

    if CONF_FAULT_DESCRIPTION in config:
        sens = await text_sensor.new_text_sensor(config[CONF_FAULT_DESCRIPTION])
        cg.add(parent.set_fault_description_sensor(sens))
