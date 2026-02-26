import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import CONF_ID

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

CONF_DHW_ENABLED = "dhw_enabled"
CONF_PUMP_MANUAL_CONTROL = "pump_manual_control"

AuroraDHWSwitch = waterfurnace_aurora_ns.class_(
    "AuroraDHWSwitch", switch.Switch, cg.Component
)
AuroraPumpManualSwitch = waterfurnace_aurora_ns.class_(
    "AuroraPumpManualSwitch", switch.Switch, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        cv.Optional(CONF_DHW_ENABLED): switch.switch_schema(
            AuroraDHWSwitch,
            icon="mdi:water-boiler",
        ).extend(cv.COMPONENT_SCHEMA),
        cv.Optional(CONF_PUMP_MANUAL_CONTROL): switch.switch_schema(
            AuroraPumpManualSwitch,
            icon="mdi:pump",
        ).extend(cv.COMPONENT_SCHEMA),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])

    if dhw_config := config.get(CONF_DHW_ENABLED):
        var = await switch.new_switch(dhw_config)
        await cg.register_component(var, dhw_config)
        cg.add(var.set_parent(parent))

    if pump_config := config.get(CONF_PUMP_MANUAL_CONTROL):
        var = await switch.new_switch(pump_config)
        await cg.register_component(var, pump_config)
        cg.add(var.set_parent(parent))
