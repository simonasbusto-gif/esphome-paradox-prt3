import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, text_sensor, binary_sensor
from esphome.const import CONF_ID

DEPENDENCIES = ["uart"]

paradox_prt3_ns = cg.esphome_ns.namespace("paradox_prt3")
ParadoxPRT3 = paradox_prt3_ns.class_("ParadoxPRT3", cg.Component, uart.UARTDevice)

CONF_UART_ID = "uart_id"
CONF_LAST_MESSAGE = "last_message"
CONF_ALARM_STATE = "alarm_state"
CONF_LAST_ERROR = "last_error"
CONF_ZONE_1 = "zone_1"
CONF_ZONE_2 = "zone_2"

CONF_READY = "ready"
CONF_TROUBLE = "trouble"
CONF_MEMORY = "memory"
CONF_STROBE = "strobe"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(ParadoxPRT3),
        cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),

        cv.Required(CONF_LAST_MESSAGE): cv.use_id(text_sensor.TextSensor),
        cv.Required(CONF_ALARM_STATE): cv.use_id(text_sensor.TextSensor),

        cv.Optional(CONF_LAST_ERROR): cv.use_id(text_sensor.TextSensor),
        cv.Optional(CONF_ZONE_1): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_ZONE_2): cv.use_id(binary_sensor.BinarySensor),

        cv.Optional(CONF_READY): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_TROUBLE): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_MEMORY): cv.use_id(binary_sensor.BinarySensor),
        cv.Optional(CONF_STROBE): cv.use_id(binary_sensor.BinarySensor),
    }
).extend(cv.COMPONENT_SCHEMA)

async def to_code(config):
    uart_comp = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], uart_comp)

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    last = await cg.get_variable(config[CONF_LAST_MESSAGE])
    state = await cg.get_variable(config[CONF_ALARM_STATE])
    cg.add(var.set_last_message(last))
    cg.add(var.set_alarm_state(state))

    if CONF_LAST_ERROR in config:
        err = await cg.get_variable(config[CONF_LAST_ERROR])
        cg.add(var.set_last_error(err))

    if CONF_ZONE_1 in config:
        z1 = await cg.get_variable(config[CONF_ZONE_1])
        cg.add(var.set_zone_1(z1))

    if CONF_ZONE_2 in config:
        z2 = await cg.get_variable(config[CONF_ZONE_2])
        cg.add(var.set_zone_2(z2))

    if CONF_READY in config:
        x = await cg.get_variable(config[CONF_READY])
        cg.add(var.set_ready(x))

    if CONF_TROUBLE in config:
        x = await cg.get_variable(config[CONF_TROUBLE])
        cg.add(var.set_trouble(x))

    if CONF_MEMORY in config:
        x = await cg.get_variable(config[CONF_MEMORY])
        cg.add(var.set_memory(x))

    if CONF_STROBE in config:
        x = await cg.get_variable(config[CONF_STROBE])
        cg.add(var.set_strobe(x))
