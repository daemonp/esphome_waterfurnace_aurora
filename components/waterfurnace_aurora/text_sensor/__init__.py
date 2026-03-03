import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

DIAGNOSTIC_TEXT_SCHEMA = text_sensor.text_sensor_schema(
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
)

# Single dictionary driving both CONFIG_SCHEMA and to_code().
# Key = YAML config key (also used to derive C++ setter via f"set_{key}_sensor").
TEXT_SENSORS = {
    "current_mode": text_sensor.text_sensor_schema(),
    "hvac_mode": text_sensor.text_sensor_schema(),
    "fan_mode": text_sensor.text_sensor_schema(),
    "fault_description": DIAGNOSTIC_TEXT_SCHEMA,
    "model_number": DIAGNOSTIC_TEXT_SCHEMA,
    "serial_number": DIAGNOSTIC_TEXT_SCHEMA,
    "fault_history": DIAGNOSTIC_TEXT_SCHEMA,
    "compressor_derate": DIAGNOSTIC_TEXT_SCHEMA,
    "compressor_safe_mode": DIAGNOSTIC_TEXT_SCHEMA,
    "compressor_alarm": DIAGNOSTIC_TEXT_SCHEMA,
    "axb_inputs": DIAGNOSTIC_TEXT_SCHEMA,
    "humidifier_mode": text_sensor.text_sensor_schema(),
    "dehumidifier_mode": text_sensor.text_sensor_schema(),
    "pump_type": DIAGNOSTIC_TEXT_SCHEMA,
    "lockout_fault_description": DIAGNOSTIC_TEXT_SCHEMA,
    "outputs_at_lockout": DIAGNOSTIC_TEXT_SCHEMA,
    "inputs_at_lockout": DIAGNOSTIC_TEXT_SCHEMA,
    "eev2_ctl": DIAGNOSTIC_TEXT_SCHEMA,
    # Configuration text sensors (gap 11)
    "brine_type": DIAGNOSTIC_TEXT_SCHEMA,
    "flow_meter_type": DIAGNOSTIC_TEXT_SCHEMA,
    "smartgrid_action": DIAGNOSTIC_TEXT_SCHEMA,
    "ha_alarm_1_action": DIAGNOSTIC_TEXT_SCHEMA,
    "ha_alarm_2_action": DIAGNOSTIC_TEXT_SCHEMA,
    "energy_phase_type": DIAGNOSTIC_TEXT_SCHEMA,
    # Compressor drive 3200-range alt diagnostics (gap 14)
    "compressor_derate_alt": DIAGNOSTIC_TEXT_SCHEMA,
    "compressor_safe_mode_alt": DIAGNOSTIC_TEXT_SCHEMA,
    "compressor_alarm_alt": DIAGNOSTIC_TEXT_SCHEMA,
    # Compressor drive EEV2 Ctl (gap 15)
    "compressor_eev2_ctl": DIAGNOSTIC_TEXT_SCHEMA,
    # Dealer information (gap 19)
    "dealer_name": DIAGNOSTIC_TEXT_SCHEMA,
    "dealer_phone": DIAGNOSTIC_TEXT_SCHEMA,
    "dealer_address_1": DIAGNOSTIC_TEXT_SCHEMA,
    "dealer_address_2": DIAGNOSTIC_TEXT_SCHEMA,
    "dealer_email": DIAGNOSTIC_TEXT_SCHEMA,
    "dealer_website": DIAGNOSTIC_TEXT_SCHEMA,
}

CONFIG_SCHEMA = cv.Schema(
    {cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora)}
).extend(
    {cv.Optional(key): schema for key, schema in TEXT_SENSORS.items()}
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])
    for key in TEXT_SENSORS:
        if conf := config.get(key):
            sens = await text_sensor.new_text_sensor(conf)
            cg.add(getattr(parent, f"set_{key}_sensor")(sens))
