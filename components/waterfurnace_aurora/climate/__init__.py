import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate
from esphome.const import CONF_ID

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@damonmaria"]

CONF_ZONE = "zone"

AuroraClimate = waterfurnace_aurora_ns.class_(
    "AuroraClimate", climate.Climate, cg.Component
)

AuroraIZ2Climate = waterfurnace_aurora_ns.class_(
    "AuroraIZ2Climate", climate.Climate, cg.Component
)


def validate_zone(value):
    """Validate zone number is 1-6."""
    value = cv.int_(value)
    if value < 1 or value > 6:
        raise cv.Invalid("Zone number must be between 1 and 6")
    return value


# Main thermostat climate schema (no zone specified)
MAIN_CLIMATE_SCHEMA = (
    climate.climate_schema(AuroraClimate)
    .extend(
        {
            cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)

# IZ2 zone climate schema (zone specified)
IZ2_CLIMATE_SCHEMA = (
    climate.climate_schema(AuroraIZ2Climate)
    .extend(
        {
            cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
            cv.Required(CONF_ZONE): validate_zone,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


def determine_schema(config):
    """Determine which schema to use based on presence of zone key."""
    if CONF_ZONE in config:
        return IZ2_CLIMATE_SCHEMA(config)
    return MAIN_CLIMATE_SCHEMA(config)


CONFIG_SCHEMA = determine_schema


async def to_code(config):
    if CONF_ZONE in config:
        # IZ2 zone climate
        var = cg.new_Pvariable(config[CONF_ID])
        await cg.register_component(var, config)
        await climate.register_climate(var, config)

        parent = await cg.get_variable(config[CONF_AURORA_ID])
        cg.add(var.set_parent(parent))
        cg.add(var.set_zone_number(config[CONF_ZONE]))
    else:
        # Main thermostat climate
        var = cg.new_Pvariable(config[CONF_ID])
        await cg.register_component(var, config)
        await climate.register_climate(var, config)

        parent = await cg.get_variable(config[CONF_AURORA_ID])
        cg.add(var.set_parent(parent))
