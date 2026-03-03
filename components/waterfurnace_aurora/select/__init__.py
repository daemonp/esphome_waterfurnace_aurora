import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import CONF_ID, ENTITY_CATEGORY_CONFIG

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

CONF_HUMIDIFIER_MODE = "humidifier_mode"
CONF_DEHUMIDIFIER_MODE = "dehumidifier_mode"
CONF_MANUAL_OPERATION = "manual_operation"
CONF_COMPRESSOR_SPEED = "compressor_speed"
CONF_BLOWER_SPEED = "blower_speed"

AuroraHumidistatSelect = waterfurnace_aurora_ns.class_(
    "AuroraHumidistatSelect", select.Select, cg.Component
)
AuroraHumidistatType = waterfurnace_aurora_ns.enum("AuroraHumidistatType", is_class=True)

AuroraManualOperationSelect = waterfurnace_aurora_ns.class_(
    "AuroraManualOperationSelect", select.Select, cg.Component
)

HUMIDISTAT_SELECT_SCHEMA = select.select_schema(
    AuroraHumidistatSelect,
    entity_category=ENTITY_CATEGORY_CONFIG,
    icon="mdi:water-percent",
).extend(cv.COMPONENT_SCHEMA)

def validate_blower_speed(value):
    """Validate blower speed: 0-12 for manual speed, or 255 for 'with_compressor' (auto)."""
    value = cv.int_(value)
    if value == 255 or 0 <= value <= 12:
        return value
    raise cv.Invalid("Blower speed must be 0-12 (manual) or 255 (auto/with_compressor)")

MANUAL_OPERATION_SELECT_SCHEMA = select.select_schema(
    AuroraManualOperationSelect,
    entity_category=ENTITY_CATEGORY_CONFIG,
    icon="mdi:wrench-cog",
).extend(cv.COMPONENT_SCHEMA).extend(
    {
        cv.Optional(CONF_COMPRESSOR_SPEED, default=8): cv.int_range(min=0, max=12),
        cv.Optional(CONF_BLOWER_SPEED, default=255): validate_blower_speed,
    }
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        cv.Optional(CONF_HUMIDIFIER_MODE): HUMIDISTAT_SELECT_SCHEMA,
        cv.Optional(CONF_DEHUMIDIFIER_MODE): HUMIDISTAT_SELECT_SCHEMA,
        cv.Optional(CONF_MANUAL_OPERATION): MANUAL_OPERATION_SELECT_SCHEMA,
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

    if manual_config := config.get(CONF_MANUAL_OPERATION):
        var = await select.new_select(
            manual_config,
            options=["Off", "Heating", "Cooling", "Heating+Aux"],
        )
        await cg.register_component(var, manual_config)
        cg.add(var.set_parent(parent))
        cg.add(var.set_compressor_speed(manual_config[CONF_COMPRESSOR_SPEED]))
        cg.add(var.set_blower_speed(manual_config[CONF_BLOWER_SPEED]))
