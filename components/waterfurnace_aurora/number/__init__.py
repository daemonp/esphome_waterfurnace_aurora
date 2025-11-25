import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_MIN_VALUE,
    CONF_MAX_VALUE,
    CONF_STEP,
    DEVICE_CLASS_TEMPERATURE,
)

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

UNIT_FAHRENHEIT = "°F"

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@damonmaria"]

CONF_DHW_SETPOINT = "dhw_setpoint"

AuroraDHWNumber = waterfurnace_aurora_ns.class_(
    "AuroraDHWNumber", number.Number, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        cv.Optional(CONF_DHW_SETPOINT): number.number_schema(
            AuroraDHWNumber,
            unit_of_measurement=UNIT_FAHRENHEIT,
            device_class=DEVICE_CLASS_TEMPERATURE,
            icon="mdi:water-thermometer",
        ).extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=100): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=140): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.float_,
            }
        ).extend(cv.COMPONENT_SCHEMA),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])

    if dhw_config := config.get(CONF_DHW_SETPOINT):
        var = await number.new_number(
            dhw_config,
            min_value=dhw_config[CONF_MIN_VALUE],
            max_value=dhw_config[CONF_MAX_VALUE],
            step=dhw_config[CONF_STEP],
        )
        await cg.register_component(var, dhw_config)
        cg.add(var.set_parent(parent))
