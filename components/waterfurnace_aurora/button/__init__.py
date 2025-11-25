import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import (
    CONF_ID,
    ENTITY_CATEGORY_CONFIG,
)

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@damonmaria"]

CONF_CLEAR_FAULT_HISTORY = "clear_fault_history"

AuroraClearFaultButton = waterfurnace_aurora_ns.class_(
    "AuroraClearFaultButton", button.Button, cg.Component
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        cv.Optional(CONF_CLEAR_FAULT_HISTORY): button.button_schema(
            AuroraClearFaultButton,
            entity_category=ENTITY_CATEGORY_CONFIG,
            icon="mdi:alert-remove",
        ).extend(cv.COMPONENT_SCHEMA),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])

    if btn_config := config.get(CONF_CLEAR_FAULT_HISTORY):
        var = await button.new_button(btn_config)
        await cg.register_component(var, btn_config)
        cg.add(var.set_parent(parent))
