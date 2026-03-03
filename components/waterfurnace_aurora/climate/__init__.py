import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, sensor, text_sensor
from esphome.const import (
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_PERCENT,
)

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID, CONF_ZONE, validate_zone

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

CONF_ZONE_PRIORITY = "zone_priority"
CONF_ZONE_SIZE = "zone_size"
CONF_ZONE_NORMALIZED_SIZE = "zone_normalized_size"

AuroraClimate = waterfurnace_aurora_ns.class_(
    "AuroraClimate", climate.Climate, cg.Component
)

CONFIG_SCHEMA = (
    climate.climate_schema(AuroraClimate)
    .extend(
        {
            cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
            cv.Optional(CONF_ZONE, default=1): validate_zone,
            cv.Optional(CONF_ZONE_PRIORITY): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_ZONE_SIZE): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_ZONE_NORMALIZED_SIZE): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=0,
                state_class=STATE_CLASS_MEASUREMENT,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
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
    if conf := config.get(CONF_ZONE_PRIORITY):
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_zone_priority_sensor(sens))
    if conf := config.get(CONF_ZONE_SIZE):
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(var.set_zone_size_sensor(sens))
    if conf := config.get(CONF_ZONE_NORMALIZED_SIZE):
        sens = await sensor.new_sensor(conf)
        cg.add(var.set_zone_normalized_size_sensor(sens))
