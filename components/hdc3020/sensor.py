import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor
from esphome.const import (
    CONF_HUMIDITY,
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_HUMIDITY,
    DEVICE_CLASS_TEMPERATURE,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    CONF_TEMPERATURE_OFFSET
)

CONF_HUMIDITY_OFFSET = "humidity_offset"

DEPENDENCIES = ["i2c"]

hdc3020_ns = cg.esphome_ns.namespace("hdc3020")
HDC3020Component = hdc3020_ns.class_(
    "HDC3020Component", cg.PollingComponent, i2c.I2CDevice
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(HDC3020Component),
            cv.Optional(CONF_TEMPERATURE): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_HUMIDITY): sensor.sensor_schema(
                unit_of_measurement=UNIT_PERCENT,
                accuracy_decimals=1,
                device_class=DEVICE_CLASS_HUMIDITY,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x44))
    .extend({
            cv.Optional(CONF_TEMPERATURE_OFFSET): cv.All(
                cv.temperature_delta,
                cv.float_range(min=-21.7, max=21.7),
            ),
            cv.Optional(CONF_HUMIDITY_OFFSET): cv.All(
                cv.float_with_unit("percentage", "(%)"),
                cv.float_range(min=-24.8, max=24.8),
            ),
            })
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature(sens))

    if CONF_HUMIDITY in config:
        sens = await sensor.new_sensor(config[CONF_HUMIDITY])
        cg.add(var.set_humidity(sens))

    if CONF_TEMPERATURE_OFFSET in config:
        cg.add(var.set_temperature_offset(config[CONF_TEMPERATURE_OFFSET]))

    if CONF_HUMIDITY_OFFSET in config:
        cg.add(var.set_humidity_offset(config[CONF_HUMIDITY_OFFSET]))
