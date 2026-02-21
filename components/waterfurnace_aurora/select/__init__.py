import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

CONF_HUMIDIFIER_MODE = "humidifier_mode"
CONF_DEHUMIDIFIER_MODE = "dehumidifier_mode"

AuroraHumidistatSelect = waterfurnace_aurora_ns.class_(
    "AuroraHumidistatSelect", select.Select, cg.Component
)
AuroraHumidistatType = waterfurnace_aurora_ns.enum("AuroraHumidistatType", is_class=True)

HUMIDISTAT_SELECT_SCHEMA = select.select_schema(
    AuroraHumidistatSelect,
    entity_category=ENTITY_CATEGORY_CONFIG,
    icon="mdi:water-percent",
).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        cv.Optional(CONF_HUMIDIFIER_MODE): HUMIDISTAT_SELECT_SCHEMA,
        cv.Optional(CONF_DEHUMIDIFIER_MODE): HUMIDISTAT_SELECT_SCHEMA,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])

    if humidifier_config := config.get(CONF_HUMIDIFIER_MODE):
        var = await select.new_select(humidifier_config, options=["Auto", "Manual"])
        await cg.register_component(var, humidifier_config)
        cg.add(var.set_parent(parent))
        cg.add(var.set_type(AuroraHumidistatType.HUMIDIFIER))

    if dehumidifier_config := config.get(CONF_DEHUMIDIFIER_MODE):
        var = await select.new_select(dehumidifier_config, options=["Auto", "Manual"])
        await cg.register_component(var, dehumidifier_config)
        cg.add(var.set_parent(parent))
        cg.add(var.set_type(AuroraHumidistatType.DEHUMIDIFIER))
