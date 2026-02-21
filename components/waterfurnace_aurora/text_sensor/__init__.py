import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor

from .. import waterfurnace_aurora_ns, WaterFurnaceAurora, CONF_AURORA_ID

DEPENDENCIES = ["waterfurnace_aurora"]
CODEOWNERS = ["@daemonp"]

# Text sensor configuration keys
CONF_CURRENT_MODE = "current_mode"
CONF_HVAC_MODE = "hvac_mode"
CONF_FAN_MODE = "fan_mode"
CONF_FAULT_DESCRIPTION = "fault_description"
CONF_MODEL_NUMBER = "model_number"
CONF_SERIAL_NUMBER = "serial_number"
CONF_FAULT_HISTORY = "fault_history"
CONF_VS_DERATE = "vs_derate"
CONF_VS_SAFE_MODE = "vs_safe_mode"
CONF_VS_ALARM = "vs_alarm"
CONF_AXB_INPUTS = "axb_inputs"
CONF_HUMIDIFIER_MODE = "humidifier_mode"
CONF_DEHUMIDIFIER_MODE = "dehumidifier_mode"
CONF_PUMP_TYPE = "pump_type"
CONF_LOCKOUT_FAULT_DESCRIPTION = "lockout_fault_description"
CONF_OUTPUTS_AT_LOCKOUT = "outputs_at_lockout"
CONF_INPUTS_AT_LOCKOUT = "inputs_at_lockout"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AURORA_ID): cv.use_id(WaterFurnaceAurora),
        cv.Optional(CONF_CURRENT_MODE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_HVAC_MODE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_FAN_MODE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_FAULT_DESCRIPTION): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_MODEL_NUMBER): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_SERIAL_NUMBER): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_FAULT_HISTORY): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_VS_DERATE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_VS_SAFE_MODE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_VS_ALARM): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_AXB_INPUTS): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_HUMIDIFIER_MODE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_DEHUMIDIFIER_MODE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_PUMP_TYPE): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_LOCKOUT_FAULT_DESCRIPTION): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_OUTPUTS_AT_LOCKOUT): text_sensor.text_sensor_schema(),
        cv.Optional(CONF_INPUTS_AT_LOCKOUT): text_sensor.text_sensor_schema(),
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_AURORA_ID])

    if CONF_CURRENT_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_CURRENT_MODE])
        cg.add(parent.set_current_mode_sensor(sens))

    if CONF_HVAC_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_HVAC_MODE])
        cg.add(parent.set_hvac_mode_sensor(sens))

    if CONF_FAN_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_FAN_MODE])
        cg.add(parent.set_fan_mode_sensor(sens))

    if CONF_FAULT_DESCRIPTION in config:
        sens = await text_sensor.new_text_sensor(config[CONF_FAULT_DESCRIPTION])
        cg.add(parent.set_fault_description_sensor(sens))

    if CONF_MODEL_NUMBER in config:
        sens = await text_sensor.new_text_sensor(config[CONF_MODEL_NUMBER])
        cg.add(parent.set_model_number_sensor(sens))

    if CONF_SERIAL_NUMBER in config:
        sens = await text_sensor.new_text_sensor(config[CONF_SERIAL_NUMBER])
        cg.add(parent.set_serial_number_sensor(sens))

    if CONF_FAULT_HISTORY in config:
        sens = await text_sensor.new_text_sensor(config[CONF_FAULT_HISTORY])
        cg.add(parent.set_fault_history_sensor(sens))

    if CONF_VS_DERATE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_VS_DERATE])
        cg.add(parent.set_vs_derate_sensor(sens))

    if CONF_VS_SAFE_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_VS_SAFE_MODE])
        cg.add(parent.set_vs_safe_mode_sensor(sens))

    if CONF_VS_ALARM in config:
        sens = await text_sensor.new_text_sensor(config[CONF_VS_ALARM])
        cg.add(parent.set_vs_alarm_sensor(sens))

    if CONF_AXB_INPUTS in config:
        sens = await text_sensor.new_text_sensor(config[CONF_AXB_INPUTS])
        cg.add(parent.set_axb_inputs_sensor(sens))

    if CONF_HUMIDIFIER_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_HUMIDIFIER_MODE])
        cg.add(parent.set_humidifier_mode_sensor(sens))

    if CONF_DEHUMIDIFIER_MODE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_DEHUMIDIFIER_MODE])
        cg.add(parent.set_dehumidifier_mode_sensor(sens))

    if CONF_PUMP_TYPE in config:
        sens = await text_sensor.new_text_sensor(config[CONF_PUMP_TYPE])
        cg.add(parent.set_pump_type_sensor(sens))

    if CONF_LOCKOUT_FAULT_DESCRIPTION in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LOCKOUT_FAULT_DESCRIPTION])
        cg.add(parent.set_lockout_fault_description_sensor(sens))

    if CONF_OUTPUTS_AT_LOCKOUT in config:
        sens = await text_sensor.new_text_sensor(config[CONF_OUTPUTS_AT_LOCKOUT])
        cg.add(parent.set_outputs_at_lockout_sensor(sens))

    if CONF_INPUTS_AT_LOCKOUT in config:
        sens = await text_sensor.new_text_sensor(config[CONF_INPUTS_AT_LOCKOUT])
        cg.add(parent.set_inputs_at_lockout_sensor(sens))
