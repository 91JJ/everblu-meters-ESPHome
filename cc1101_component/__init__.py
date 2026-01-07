import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import spi, sensor, time, text_sensor
from esphome import pins
from esphome.const import CONF_ID, CONF_CS_PIN

AUTO_LOAD = ['sensor', 'time', 'text_sensor']
DEPENDENCIES = ['spi']

cc1101_component_ns = cg.esphome_ns.namespace('cc1101_component')
CC1101Component = cc1101_component_ns.class_('CC1101Component', cg.Component, spi.SPIDevice)

CONF_GDO0_PIN = 'gdo0_pin'
CONF_FREQUENCY = 'frequency'
CONF_METER_YEAR = 'meter_year'
CONF_METER_SERIAL = 'meter_serial'
CONF_LITERS = 'liters'
CONF_COUNTER = 'counter'
CONF_BATTERY = 'battery'
CONF_TIME_START = 'time_start'
CONF_TIME_END = 'time_end'
CONF_RSSI = 'rssi'
CONF_TUNED_FREQUENCY = 'tuned_frequency'
CONF_JSON_SENSOR = 'json_sensor'
CONF_TIMESTAMP_SENSOR = 'timestamp_sensor'
CONF_TIME_ID = 'time_id'

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(CC1101Component),
    cv.Required(CONF_GDO0_PIN): pins.gpio_input_pin_schema,
    cv.Optional(CONF_FREQUENCY, default=433.82): cv.float_,
    cv.Optional(CONF_METER_YEAR, default=12): cv.int_range(min=0, max=99),
    cv.Optional(CONF_METER_SERIAL, default=123456): cv.uint32_t,
    cv.Optional(CONF_LITERS): sensor.sensor_schema(
        unit_of_measurement="L",
        icon="mdi:water",
        accuracy_decimals=0,
        device_class="water",
        state_class="total_increasing",
    ),
    cv.Optional(CONF_COUNTER): sensor.sensor_schema(
        unit_of_measurement="reads",
        icon="mdi:counter",
        accuracy_decimals=0,
        state_class="measurement",
    ),
    cv.Optional(CONF_BATTERY): sensor.sensor_schema(
        unit_of_measurement="%",
        icon="mdi:battery",
        accuracy_decimals=0,
        device_class="battery",
        state_class="measurement",
    ),
    cv.Optional(CONF_TIME_START): sensor.sensor_schema(
        unit_of_measurement="h",
        icon="mdi:clock",
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_TIME_END): sensor.sensor_schema(
        unit_of_measurement="h",
        icon="mdi:clock",
        accuracy_decimals=0,
    ),
    cv.Optional(CONF_RSSI): sensor.sensor_schema(
        unit_of_measurement="dBm",
        icon="mdi:signal",
        accuracy_decimals=0,
        state_class="measurement",
    ),
    cv.Optional(CONF_TUNED_FREQUENCY): sensor.sensor_schema(
        unit_of_measurement="MHz",
        icon="mdi:radio",
        accuracy_decimals=6,
        state_class="measurement",
    ),
    cv.Optional(CONF_JSON_SENSOR): text_sensor.text_sensor_schema(),
    cv.Optional(CONF_TIMESTAMP_SENSOR): text_sensor.text_sensor_schema(),
    cv.Required(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
}).extend(cv.COMPONENT_SCHEMA).extend(spi.spi_device_schema()).extend({
    cv.Required(CONF_CS_PIN): pins.gpio_output_pin_schema,
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    cg.add(var.set_gdo0_pin(await cg.gpio_pin_expression(config[CONF_GDO0_PIN])))
    cg.add(var.set_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_meter_year(config[CONF_METER_YEAR]))
    cg.add(var.set_meter_serial(config[CONF_METER_SERIAL]))

    if CONF_LITERS in config:
        sens = await sensor.new_sensor(config[CONF_LITERS])
        cg.add(var.set_liters_sensor(sens))
    if CONF_COUNTER in config:
        sens = await sensor.new_sensor(config[CONF_COUNTER])
        cg.add(var.set_counter_sensor(sens))
    if CONF_BATTERY in config:
        sens = await sensor.new_sensor(config[CONF_BATTERY])
        cg.add(var.set_battery_sensor(sens))
    if CONF_TIME_START in config:
        sens = await sensor.new_sensor(config[CONF_TIME_START])
        cg.add(var.set_time_start_sensor(sens))
    if CONF_TIME_END in config:
        sens = await sensor.new_sensor(config[CONF_TIME_END])
        cg.add(var.set_time_end_sensor(sens))
    if CONF_RSSI in config:
        sens = await sensor.new_sensor(config[CONF_RSSI])
        cg.add(var.set_rssi_sensor(sens))
    if CONF_TUNED_FREQUENCY in config:
        sens = await sensor.new_sensor(config[CONF_TUNED_FREQUENCY])
        cg.add(var.set_tuned_frequency_sensor(sens))
    if CONF_JSON_SENSOR in config:
        sens = await text_sensor.new_text_sensor(config[CONF_JSON_SENSOR])
        cg.add(var.set_json_sensor(sens))
    if CONF_TIMESTAMP_SENSOR in config:
        sens = await text_sensor.new_text_sensor(config[CONF_TIMESTAMP_SENSOR])
        cg.add(var.set_timestamp_sensor(sens))
    time_ = await cg.get_variable(config[CONF_TIME_ID])
    cg.add(var.set_time(time_))
