import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    CONF_ID,
    CONF_MIN_VALUE,
    CONF_MAX_VALUE,
    CONF_STEP,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    UNIT_PERCENT,
    UNIT_MINUTE,
)

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

UNIT_FAHRENHEIT = "°F"

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@damonmaria"]

# Configuration keys
CONF_DHW_SETPOINT = "dhw_setpoint"
CONF_BLOWER_ONLY_SPEED = "blower_only_speed"
CONF_LO_COMPRESSOR_SPEED = "lo_compressor_speed"
CONF_HI_COMPRESSOR_SPEED = "hi_compressor_speed"
CONF_AUX_HEAT_SPEED = "aux_heat_speed"
CONF_PUMP_SPEED = "pump_speed"
CONF_PUMP_MIN_SPEED = "pump_min_speed"
CONF_PUMP_MAX_SPEED = "pump_max_speed"
CONF_FAN_INTERMITTENT_ON = "fan_intermittent_on"
CONF_FAN_INTERMITTENT_OFF = "fan_intermittent_off"
CONF_HUMIDIFICATION_TARGET = "humidification_target"
CONF_DEHUMIDIFICATION_TARGET = "dehumidification_target"

# C++ classes
AuroraDHWNumber = waterfurnace_aurora_ns.class_(
    "AuroraDHWNumber", number.Number, cg.Component
)
AuroraNumber = waterfurnace_aurora_ns.class_(
    "AuroraNumber", number.Number, cg.Component
)
AuroraNumberType = waterfurnace_aurora_ns.enum("AuroraNumberType")

# Number type enum values
AURORA_NUMBER_TYPES = {
    CONF_BLOWER_ONLY_SPEED: AuroraNumberType.BLOWER_ONLY_SPEED,
    CONF_LO_COMPRESSOR_SPEED: AuroraNumberType.LO_COMPRESSOR_SPEED,
    CONF_HI_COMPRESSOR_SPEED: AuroraNumberType.HI_COMPRESSOR_SPEED,
    CONF_AUX_HEAT_SPEED: AuroraNumberType.AUX_HEAT_SPEED,
    CONF_PUMP_SPEED: AuroraNumberType.PUMP_SPEED,
    CONF_PUMP_MIN_SPEED: AuroraNumberType.PUMP_MIN_SPEED,
    CONF_PUMP_MAX_SPEED: AuroraNumberType.PUMP_MAX_SPEED,
    CONF_FAN_INTERMITTENT_ON: AuroraNumberType.FAN_INTERMITTENT_ON,
    CONF_FAN_INTERMITTENT_OFF: AuroraNumberType.FAN_INTERMITTENT_OFF,
    CONF_HUMIDIFICATION_TARGET: AuroraNumberType.HUMIDIFICATION_TARGET,
    CONF_DEHUMIDIFICATION_TARGET: AuroraNumberType.DEHUMIDIFICATION_TARGET,
}

# Schema for ECM blower speeds (1-12)
BLOWER_SPEED_SCHEMA = number.number_schema(
    AuroraNumber,
    icon="mdi:fan",
).extend(
    {
        cv.Optional(CONF_MIN_VALUE, default=1): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=12): cv.float_,
        cv.Optional(CONF_STEP, default=1): cv.float_,
    }
).extend(cv.COMPONENT_SCHEMA)

# Schema for pump speeds (1-100%)
PUMP_SPEED_SCHEMA = number.number_schema(
    AuroraNumber,
    unit_of_measurement=UNIT_PERCENT,
    icon="mdi:pump",
).extend(
    {
        cv.Optional(CONF_MIN_VALUE, default=1): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=100): cv.float_,
        cv.Optional(CONF_STEP, default=1): cv.float_,
    }
).extend(cv.COMPONENT_SCHEMA)

# Schema for fan intermittent on time (0, 5, 10, 15, 20, 25 minutes)
FAN_ON_TIME_SCHEMA = number.number_schema(
    AuroraNumber,
    unit_of_measurement="min",
    icon="mdi:timer",
).extend(
    {
        cv.Optional(CONF_MIN_VALUE, default=0): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=25): cv.float_,
        cv.Optional(CONF_STEP, default=5): cv.float_,
    }
).extend(cv.COMPONENT_SCHEMA)

# Schema for fan intermittent off time (5, 10, 15, 20, 25, 30, 35, 40 minutes)
FAN_OFF_TIME_SCHEMA = number.number_schema(
    AuroraNumber,
    unit_of_measurement="min",
    icon="mdi:timer-off",
).extend(
    {
        cv.Optional(CONF_MIN_VALUE, default=5): cv.float_,
        cv.Optional(CONF_MAX_VALUE, default=40): cv.float_,
        cv.Optional(CONF_STEP, default=5): cv.float_,
    }
).extend(cv.COMPONENT_SCHEMA)

# Schema for humidity targets
HUMIDITY_TARGET_SCHEMA = number.number_schema(
    AuroraNumber,
    unit_of_measurement=UNIT_PERCENT,
    device_class=DEVICE_CLASS_HUMIDITY,
    icon="mdi:water-percent",
).extend(cv.COMPONENT_SCHEMA)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        # DHW setpoint (existing)
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
        # Blower speed settings
        cv.Optional(CONF_BLOWER_ONLY_SPEED): BLOWER_SPEED_SCHEMA,
        cv.Optional(CONF_LO_COMPRESSOR_SPEED): BLOWER_SPEED_SCHEMA,
        cv.Optional(CONF_HI_COMPRESSOR_SPEED): BLOWER_SPEED_SCHEMA,
        cv.Optional(CONF_AUX_HEAT_SPEED): BLOWER_SPEED_SCHEMA,
        # Pump speed settings
        cv.Optional(CONF_PUMP_SPEED): PUMP_SPEED_SCHEMA,
        cv.Optional(CONF_PUMP_MIN_SPEED): PUMP_SPEED_SCHEMA,
        cv.Optional(CONF_PUMP_MAX_SPEED): PUMP_SPEED_SCHEMA,
        # Fan intermittent timing
        cv.Optional(CONF_FAN_INTERMITTENT_ON): FAN_ON_TIME_SCHEMA,
        cv.Optional(CONF_FAN_INTERMITTENT_OFF): FAN_OFF_TIME_SCHEMA,
        # Humidity targets
        cv.Optional(CONF_HUMIDIFICATION_TARGET): HUMIDITY_TARGET_SCHEMA.extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=15): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=50): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.float_,
            }
        ),
        cv.Optional(CONF_DEHUMIDIFICATION_TARGET): HUMIDITY_TARGET_SCHEMA.extend(
            {
                cv.Optional(CONF_MIN_VALUE, default=35): cv.float_,
                cv.Optional(CONF_MAX_VALUE, default=65): cv.float_,
                cv.Optional(CONF_STEP, default=1): cv.float_,
            }
        ),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])

    # DHW setpoint (special case - uses AuroraDHWNumber)
    if dhw_config := config.get(CONF_DHW_SETPOINT):
        var = await number.new_number(
            dhw_config,
            min_value=dhw_config[CONF_MIN_VALUE],
            max_value=dhw_config[CONF_MAX_VALUE],
            step=dhw_config[CONF_STEP],
        )
        await cg.register_component(var, dhw_config)
        cg.add(var.set_parent(parent))

    # Generic number controls
    for conf_key, number_type in AURORA_NUMBER_TYPES.items():
        if num_config := config.get(conf_key):
            var = await number.new_number(
                num_config,
                min_value=num_config[CONF_MIN_VALUE],
                max_value=num_config[CONF_MAX_VALUE],
                step=num_config[CONF_STEP],
            )
            await cg.register_component(var, num_config)
            cg.add(var.set_parent(parent))
            cg.add(var.set_type(number_type))
