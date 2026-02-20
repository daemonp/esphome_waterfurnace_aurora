import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import uart
from esphome.const import (
    CONF_ID,
    CONF_ADDRESS,
    CONF_UPDATE_INTERVAL,
)

CODEOWNERS = ["@damonmaria"]
DEPENDENCIES = ["uart"]
AUTO_LOAD = ["sensor", "binary_sensor", "text_sensor"]
MULTI_CONF = True

CONF_AURORA_ID = "aurora_id"
CONF_FLOW_CONTROL_PIN = "flow_control_pin"
CONF_READ_RETRIES = "read_retries"
CONF_HAS_AXB = "has_axb"
CONF_HAS_VS_DRIVE = "has_vs_drive"
CONF_HAS_IZ2 = "has_iz2"
CONF_NUM_IZ2_ZONES = "num_iz2_zones"

waterfurnace_aurora_ns = cg.esphome_ns.namespace("waterfurnace_aurora")
WaterFurnaceAurora = waterfurnace_aurora_ns.class_(
    "WaterFurnaceAurora", cg.PollingComponent, uart.UARTDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(WaterFurnaceAurora),
            cv.Optional(CONF_ADDRESS, default=1): cv.int_range(min=1, max=247),
            cv.Optional(CONF_UPDATE_INTERVAL, default="5s"): cv.update_interval,
            cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_READ_RETRIES, default=2): cv.int_range(min=0, max=10),
            # Hardware override options (auto-detected if not specified)
            cv.Optional(CONF_HAS_AXB): cv.boolean,
            cv.Optional(CONF_HAS_VS_DRIVE): cv.boolean,
            cv.Optional(CONF_HAS_IZ2): cv.boolean,
            cv.Optional(CONF_NUM_IZ2_ZONES): cv.int_range(min=0, max=6),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    cg.add(var.set_address(config[CONF_ADDRESS]))
    
    if CONF_FLOW_CONTROL_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(pin))
    
    cg.add(var.set_read_retries(config[CONF_READ_RETRIES]))

    # Hardware override options
    if CONF_HAS_AXB in config:
        cg.add(var.set_has_axb_override(config[CONF_HAS_AXB]))
    if CONF_HAS_VS_DRIVE in config:
        cg.add(var.set_has_vs_drive_override(config[CONF_HAS_VS_DRIVE]))
    if CONF_HAS_IZ2 in config:
        cg.add(var.set_has_iz2_override(config[CONF_HAS_IZ2]))
    if CONF_NUM_IZ2_ZONES in config:
        cg.add(var.set_num_iz2_zones_override(config[CONF_NUM_IZ2_ZONES]))
