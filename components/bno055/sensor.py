import esphome.codegen as cg
from esphome.components import i2c, sensor, binary_sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_TEMPERATURE,
    DEVICE_CLASS_TEMPERATURE,
    ICON_BRIEFCASE_DOWNLOAD,
    ICON_SCREEN_ROTATION,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_DEGREE_PER_SECOND,
    UNIT_METER_PER_SECOND_SQUARED,
    UNIT_DEGREES,
    UNIT_MICROTESLA,
    UNIT_MPS,
    UNIT_METER,
)

DEPENDENCIES = ["i2c"]

CONF_ACCEL_X = "accel_x"
CONF_ACCEL_Y = "accel_y"
CONF_ACCEL_Z = "accel_z"
CONF_GYRO_X = "gyro_x"
CONF_GYRO_Y = "gyro_y"
CONF_GYRO_Z = "gyro_z"
CONF_MAG_X = "mag_x"
CONF_MAG_Y = "mag_y"
CONF_MAG_Z = "mag_z"
CONF_EULER_X = "euler_x"
CONF_EULER_Y = "euler_y"
CONF_EULER_Z = "euler_z"
CONF_LINEAR_ACCEL_X = "linear_accel_x"
CONF_LINEAR_ACCEL_Y = "linear_accel_y"
CONF_LINEAR_ACCEL_Z = "linear_accel_z"
CONF_GRAVITY_X = "gravity_x"
CONF_GRAVITY_Y = "gravity_y"
CONF_GRAVITY_Z = "gravity_z"
CONF_QUATERNION_W = "quaternion_w"
CONF_QUATERNION_X = "quaternion_x"
CONF_QUATERNION_Y = "quaternion_y"
CONF_QUATERNION_Z = "quaternion_z"
CONF_MAGNETIC_NORTH = "magnetic_north"
CONF_TRUE_HEADING = "true_heading"
CONF_MAGNETIC_DECLINATION = "magnetic_declination"
CONF_CALIBRATION_SYSTEM = "calibration_system"
CONF_CALIBRATION_GYRO = "calibration_gyro"
CONF_CALIBRATION_ACCEL = "calibration_accel"
CONF_CALIBRATION_MAG = "calibration_mag"
CONF_CALIBRATION_COMPLETE = "calibration_complete"
CONF_SPEED = "speed"
CONF_DISTANCE = "distance"

bno055_ns = cg.esphome_ns.namespace("bno055")
BNO055Component = bno055_ns.class_(
    "BNO055Component", cg.PollingComponent, i2c.I2CDevice
)

accel_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=3,
    state_class=STATE_CLASS_MEASUREMENT,
)
gyro_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREE_PER_SECOND,
    icon=ICON_SCREEN_ROTATION,
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)
mag_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_MICROTESLA,
    icon="mdi:compass",
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)
euler_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    icon="mdi:compass",
    accuracy_decimals=2,
    state_class=STATE_CLASS_MEASUREMENT,
)
linear_accel_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=3,
    state_class=STATE_CLASS_MEASUREMENT,
)
gravity_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER_PER_SECOND_SQUARED,
    icon=ICON_BRIEFCASE_DOWNLOAD,
    accuracy_decimals=3,
    state_class=STATE_CLASS_MEASUREMENT,
)
quaternion_schema = sensor.sensor_schema(
    accuracy_decimals=4,
    state_class=STATE_CLASS_MEASUREMENT,
)
temperature_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_CELSIUS,
    accuracy_decimals=1,
    device_class=DEVICE_CLASS_TEMPERATURE,
    state_class=STATE_CLASS_MEASUREMENT,
)
magnetic_north_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    icon="mdi:compass",
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)
true_heading_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_DEGREES,
    icon="mdi:compass",
    accuracy_decimals=1,
    state_class=STATE_CLASS_MEASUREMENT,
)
calibration_schema = sensor.sensor_schema(
    accuracy_decimals=0,
    state_class=STATE_CLASS_MEASUREMENT,
)

calibration_complete_schema = binary_sensor.binary_sensor_schema(
    icon="mdi:check-circle",
)

speed_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_MPS,
    icon="mdi:speedometer",
    accuracy_decimals=3,
    state_class=STATE_CLASS_MEASUREMENT,
)

distance_schema = sensor.sensor_schema(
    unit_of_measurement=UNIT_METER,
    icon="mdi:map-marker-distance",
    accuracy_decimals=3,
    state_class=STATE_CLASS_MEASUREMENT,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(BNO055Component),
            cv.Optional(CONF_ACCEL_X): accel_schema,
            cv.Optional(CONF_ACCEL_Y): accel_schema,
            cv.Optional(CONF_ACCEL_Z): accel_schema,
            cv.Optional(CONF_GYRO_X): gyro_schema,
            cv.Optional(CONF_GYRO_Y): gyro_schema,
            cv.Optional(CONF_GYRO_Z): gyro_schema,
            cv.Optional(CONF_MAG_X): mag_schema,
            cv.Optional(CONF_MAG_Y): mag_schema,
            cv.Optional(CONF_MAG_Z): mag_schema,
            cv.Optional(CONF_EULER_X): euler_schema,
            cv.Optional(CONF_EULER_Y): euler_schema,
            cv.Optional(CONF_EULER_Z): euler_schema,
            cv.Optional(CONF_LINEAR_ACCEL_X): linear_accel_schema,
            cv.Optional(CONF_LINEAR_ACCEL_Y): linear_accel_schema,
            cv.Optional(CONF_LINEAR_ACCEL_Z): linear_accel_schema,
            cv.Optional(CONF_GRAVITY_X): gravity_schema,
            cv.Optional(CONF_GRAVITY_Y): gravity_schema,
            cv.Optional(CONF_GRAVITY_Z): gravity_schema,
            cv.Optional(CONF_QUATERNION_W): quaternion_schema,
            cv.Optional(CONF_QUATERNION_X): quaternion_schema,
            cv.Optional(CONF_QUATERNION_Y): quaternion_schema,
            cv.Optional(CONF_QUATERNION_Z): quaternion_schema,
            cv.Optional(CONF_TEMPERATURE): temperature_schema,
            cv.Optional(CONF_MAGNETIC_NORTH): magnetic_north_schema,
            cv.Optional(CONF_TRUE_HEADING): true_heading_schema,
            cv.Optional(CONF_SPEED): speed_schema,
            cv.Optional(CONF_DISTANCE): distance_schema,
            cv.Optional(CONF_CALIBRATION_SYSTEM): calibration_schema,
            cv.Optional(CONF_CALIBRATION_GYRO): calibration_schema,
            cv.Optional(CONF_CALIBRATION_ACCEL): calibration_schema,
            cv.Optional(CONF_CALIBRATION_MAG): calibration_schema,
            cv.Optional(CONF_CALIBRATION_COMPLETE): calibration_complete_schema,
            cv.Optional(CONF_MAGNETIC_DECLINATION, default=0.0): cv.float_,
        }
    )
    .extend(cv.polling_component_schema("60s"))
    .extend(i2c.i2c_device_schema(0x28))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    for axis in ["x", "y", "z"]:
        for sensor_type in ["accel", "gyro", "mag", "euler", "linear_accel", "gravity"]:
            key = f"{sensor_type}_{axis}"
            if key in config:
                sens = await sensor.new_sensor(config[key])
                cg.add(getattr(var, f"set_{sensor_type}_{axis}_sensor")(sens))

    for quat in ["w", "x", "y", "z"]:
        key = f"quaternion_{quat}"
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_quaternion_{quat}_sensor")(sens))

    if CONF_TEMPERATURE in config:
        sens = await sensor.new_sensor(config[CONF_TEMPERATURE])
        cg.add(var.set_temperature_sensor(sens))

    if CONF_MAGNETIC_NORTH in config:
        sens = await sensor.new_sensor(config[CONF_MAGNETIC_NORTH])
        cg.add(var.set_magnetic_north_sensor(sens))

    if CONF_TRUE_HEADING in config:
        sens = await sensor.new_sensor(config[CONF_TRUE_HEADING])
        cg.add(var.set_true_heading_sensor(sens))

    if CONF_CALIBRATION_SYSTEM in config:
        sens = await sensor.new_sensor(config[CONF_CALIBRATION_SYSTEM])
        cg.add(var.set_calibration_system_sensor(sens))
    
    if CONF_CALIBRATION_GYRO in config:
        sens = await sensor.new_sensor(config[CONF_CALIBRATION_GYRO])
        cg.add(var.set_calibration_gyro_sensor(sens))
    
    if CONF_CALIBRATION_ACCEL in config:
        sens = await sensor.new_sensor(config[CONF_CALIBRATION_ACCEL])
        cg.add(var.set_calibration_accel_sensor(sens))
    
    if CONF_CALIBRATION_MAG in config:
        sens = await sensor.new_sensor(config[CONF_CALIBRATION_MAG])
        cg.add(var.set_calibration_mag_sensor(sens))

    if CONF_CALIBRATION_COMPLETE in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_CALIBRATION_COMPLETE])
        cg.add(var.set_calibration_complete_binary_sensor(sens))

    if CONF_SPEED in config:
        sens = await sensor.new_sensor(config[CONF_SPEED])
        cg.add(var.set_speed_sensor(sens))

    if CONF_DISTANCE in config:
        sens = await sensor.new_sensor(config[CONF_DISTANCE])
        cg.add(var.set_distance_sensor(sens))

    cg.add(var.set_magnetic_declination(config[CONF_MAGNETIC_DECLINATION])) 
