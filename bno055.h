#pragma once

#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace bno055 {

class BNO055Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;

  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; }
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }
  void set_mag_x_sensor(sensor::Sensor *mag_x_sensor) { mag_x_sensor_ = mag_x_sensor; }
  void set_mag_y_sensor(sensor::Sensor *mag_y_sensor) { mag_y_sensor_ = mag_y_sensor; }
  void set_mag_z_sensor(sensor::Sensor *mag_z_sensor) { mag_z_sensor_ = mag_z_sensor; }
  void set_euler_h_sensor(sensor::Sensor *euler_h_sensor) { euler_h_sensor_ = euler_h_sensor; }
  void set_euler_r_sensor(sensor::Sensor *euler_r_sensor) { euler_r_sensor_ = euler_r_sensor; }
  void set_euler_p_sensor(sensor::Sensor *euler_p_sensor) { euler_p_sensor_ = euler_p_sensor; }
  void set_linear_accel_x_sensor(sensor::Sensor *linear_accel_x_sensor) { linear_accel_x_sensor_ = linear_accel_x_sensor; }
  void set_linear_accel_y_sensor(sensor::Sensor *linear_accel_y_sensor) { linear_accel_y_sensor_ = linear_accel_y_sensor; }
  void set_linear_accel_z_sensor(sensor::Sensor *linear_accel_z_sensor) { linear_accel_z_sensor_ = linear_accel_z_sensor; }
  void set_gravity_x_sensor(sensor::Sensor *gravity_x_sensor) { gravity_x_sensor_ = gravity_x_sensor; }
  void set_gravity_y_sensor(sensor::Sensor *gravity_y_sensor) { gravity_y_sensor_ = gravity_y_sensor; }
  void set_gravity_z_sensor(sensor::Sensor *gravity_z_sensor) { gravity_z_sensor_ = gravity_z_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_compass_sensor(sensor::Sensor *compass_sensor) { compass_sensor_ = compass_sensor; }

 protected:
  bool read_sensor_data_();
  bool write_byte_(uint8_t register_address, uint8_t value);
  bool read_byte_(uint8_t register_address, uint8_t *value);
  bool read_bytes_(uint8_t register_address, uint8_t *data, size_t len);

  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
  sensor::Sensor *mag_x_sensor_{nullptr};
  sensor::Sensor *mag_y_sensor_{nullptr};
  sensor::Sensor *mag_z_sensor_{nullptr};
  sensor::Sensor *euler_h_sensor_{nullptr};
  sensor::Sensor *euler_r_sensor_{nullptr};
  sensor::Sensor *euler_p_sensor_{nullptr};
  sensor::Sensor *linear_accel_x_sensor_{nullptr};
  sensor::Sensor *linear_accel_y_sensor_{nullptr};
  sensor::Sensor *linear_accel_z_sensor_{nullptr};
  sensor::Sensor *gravity_x_sensor_{nullptr};
  sensor::Sensor *gravity_y_sensor_{nullptr};
  sensor::Sensor *gravity_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *compass_sensor_{nullptr};

  enum ErrorCode {
    NONE = 0,
    COMMUNICATION_FAILED,
    ID_MISMATCH,
    INITIALIZATION_FAILED,
  } error_code_{NONE};
};

}  // namespace bno055
}  // namespace esphome 