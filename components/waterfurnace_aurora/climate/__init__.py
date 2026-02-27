import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID, CONF_ZONE, validate_zone

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

AuroraClimate = waterfurnace_aurora_ns.class_(
    "AuroraClimate", climate.Climate, cg.Component
)

CONFIG_SCHEMA = (
    climate.climate_schema(AuroraClimate)
    .extend(
        {
            cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
            cv.Optional(CONF_ZONE, default=1): validate_zone,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_AURORA_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_zone(config[CONF_ZONE]))
