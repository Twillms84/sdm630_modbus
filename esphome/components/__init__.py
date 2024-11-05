#import esphome.codegen as cg
#import esphome.config_validation as cv
#from esphome.const import CONF_ADDRESS, CONF_FLOW_CONTROL_PIN, CONF_TOPIC, CONF_ID
#from esphome.cpp_helpers import gpio_pin_expression
from esphome import pins
#from esphome.components import uart
#from esphome.const import CONF_ID
#from esphome import config_validation as cv

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_FLOW_CONTROL_PIN
from esphome.components import uart
#from esphome.components import gpio
from esphome.cpp_helpers import gpio_pin_expression

DEPENDENCIES = ["uart"]

#sdm630_modbus_ns = cg.esphome_ns.namespace("sdm630_modbus")
#SDM630Modbus = sdm630_modbus_ns.class_(
#    "SDM630Modbus", cg.Component, uart.UARTDevice
#)

sdm630_modbus_ns = cg.esphome_ns.namespace('sdm630_modbus')
SDM630Modbus = sdm630_modbus_ns.class_('SDM630Modbus', cg.Component, uart.UARTDevice)

#CONFIG_SCHEMA = (
#    cv.Schema(
#        {
#        cv.GenerateID(): cv.declare_id(SDM630Modbus),
#        cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
#        }
#    )
#    .extend(cv.COMPONENT_SCHEMA)
#    .extend(uart.UART_DEVICE_SCHEMA)
#)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(SDM630Modbus),
    cv.Optional(CONF_FLOW_CONTROL_PIN): pins.gpio_output_pin_schema,
    cv.Required('uart_id'): cv.use_id(uart.UARTComponent),

}).extend(cv.COMPONENT_SCHEMA)


#async def to_code(config):
#    var = cg.new_Pvariable(config[CONF_ID])
#    await cg.register_component(var, config)
#    await uart.register_uart_device(var, config)  # Registriert das Gerät als UART

#    if CONF_FLOW_CONTROL_PIN in config:
#        pin = await gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
#        cg.add(var.set_flow_control_pin(pin))

# Add a function to make the component available for ESPHome
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    # UART-Objekt korrekt registrieren und setzen
    uart_comp = await cg.get_variable(config['uart_id'])
    cg.add(var.set_uart_parent(uart_comp))  # set_uart_parent statt set_uart_id verwenden
    
    if CONF_FLOW_CONTROL_PIN in config:
        flow_control_pin = await cg.gpio_pin_expression(config[CONF_FLOW_CONTROL_PIN])
        cg.add(var.set_flow_control_pin(flow_control_pin))
    
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    
    