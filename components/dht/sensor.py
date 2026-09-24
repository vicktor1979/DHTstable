from esphome import pins
import esphome.codegen as cg
from esphome.components import sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_HUMIDITY,
    CONF_ID,
    CONF_MODEL,
    CONF_PIN,
    CONF_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
)

CODEOWNERS = ["@vicktor1979"]

dht_ns = cg.esphome_ns.namespace("dht")
DHT = dht_ns.class_("DHT", cg.PollingComponent)
DHTModel = dht_ns.enum("DHTModel")

DHT_MODELS = {
    "AUTO_DETECT": DHTModel.DHT_MODEL_AUTO_DETECT,
    "DHT11": DHTModel.DHT_MODEL_DHT11,
    "DHT22": DHTModel.DHT_MODEL_DHT22,
    "AM2120": DHTModel.DHT_MODEL_AM2120,
    "AM2302": DHTModel.DHT_MODEL_AM2302,
    "RHT03": DHTModel.DHT_MODEL_RHT03,
    "SI7021": DHTModel.DHT_MODEL_SI7021,
    "DHT22_TYPE2": DHTModel.DHT_MODEL_DHT22_TYPE2,
}

_DHT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(DHT),
        cv.Required(CONF_PIN): pins.internal_gpio_input_pullup_pin_schema,
        cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_HUMIDITY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional(CONF_MODEL, default="AUTO_DETECT"): cv.enum(
            DHT_MODELS, upper=True
        ),
    }
).extend(cv.polling_component_schema("60s"))

# DHTStable uses Arduino.h and the Arduino GPIO API.
CONFIG_SCHEMA = cv.All(_DHT_SCHEMA, cv.only_with_arduino)


async def to_code(config):
    # Use the modified DHTStable library from our own fork/branch.
    cg.add_library(
        "DHTStable",
        None,
        "https://github.com/vicktor1979/DHTstable.git#esphome",
    )

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    pin = await cg.gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
    cg.add(var.set_dht_model(config[CONF_MODEL]))

    if temperature_config := config.get(CONF_TEMPERATURE):
        sens = await sensor.new_sensor(temperature_config)
        cg.add(var.set_temperature_sensor(sens))

    if humidity_config := config.get(CONF_HUMIDITY):
        sens = await sensor.new_sensor(humidity_config)
        cg.add(var.set_humidity_sensor(sens))
