import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import water_heater

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

# In ESPHome <= 2026.1.x, WaterHeater already inherits from Component and
# register_water_heater() calls register_component().  In 2026.2+, Component
# was removed from WaterHeater, so we must add it ourselves.
_WH_HAS_COMPONENT = water_heater.WaterHeater.inherits_from(cg.Component)

_bases = (
    (water_heater.WaterHeater,)
    if _WH_HAS_COMPONENT
    else (water_heater.WaterHeater, cg.Component)
)

AuroraWaterHeater = waterfurnace_aurora_ns.class_(
    "AuroraWaterHeater", *_bases
)

CONFIG_SCHEMA = water_heater.water_heater_schema(
    AuroraWaterHeater, icon="mdi:water-boiler"
).extend(
    {
        cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
    }
)

if not _WH_HAS_COMPONENT:
    CONFIG_SCHEMA = CONFIG_SCHEMA.extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await water_heater.new_water_heater(config)

    # In 2026.2+ register_water_heater() no longer calls register_component(),
    # so we must do it ourselves.
    if not _WH_HAS_COMPONENT:
        await cg.register_component(var, config)

    parent = await cg.get_variable(config[CONF_AURORA_ID])
    cg.add(var.set_parent(parent))
