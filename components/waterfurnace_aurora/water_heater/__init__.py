import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import water_heater
from esphome.const import CONF_ID

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

AuroraWaterHeater = waterfurnace_aurora_ns.class_(
    "AuroraWaterHeater", water_heater.WaterHeater, cg.Component
)

CONFIG_SCHEMA = (
    water_heater.water_heater_schema(AuroraWaterHeater, icon="mdi:water-boiler")
    .extend(
        {
            cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
)


async def to_code(config):
    var = await water_heater.new_water_heater(config)

    parent = await cg.get_variable(config[CONF_AURORA_ID])
    cg.add(var.set_parent(parent))
