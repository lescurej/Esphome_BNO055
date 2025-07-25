#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace bno055 {

class BNO055Component : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  void update() override;
  float get_setup_priority() const override;

  void set_accel_x_sensor(sensor::Sensor *accel_x_sensor) { accel_x_sensor_ = accel_x_sensor; }
  void set_accel_y_sensor(sensor::Sensor *accel_y_sensor) { accel_y_sensor_ = accel_y_sensor; }
  void set_accel_z_sensor(sensor::Sensor *accel_z_sensor) { accel_z_sensor_ = accel_z_sensor; }
  void set_gyro_x_sensor(sensor::Sensor *gyro_x_sensor) { gyro_x_sensor_ = gyro_x_sensor; }
  void set_gyro_y_sensor(sensor::Sensor *gyro_y_sensor) { gyro_y_sensor_ = gyro_y_sensor; }
  void set_gyro_z_sensor(sensor::Sensor *gyro_z_sensor) { gyro_z_sensor_ = gyro_z_sensor; }
  void set_mag_x_sensor(sensor::Sensor *mag_x_sensor) { mag_x_sensor_ = mag_x_sensor; }
  void set_mag_y_sensor(sensor::Sensor *mag_y_sensor) { mag_y_sensor_ = mag_y_sensor; }
  void set_mag_z_sensor(sensor::Sensor *mag_z_sensor) { mag_z_sensor_ = mag_z_sensor; }
  void set_euler_x_sensor(sensor::Sensor *euler_x_sensor) { euler_x_sensor_ = euler_x_sensor; }
  void set_euler_y_sensor(sensor::Sensor *euler_y_sensor) { euler_y_sensor_ = euler_y_sensor; }
  void set_euler_z_sensor(sensor::Sensor *euler_z_sensor) { euler_z_sensor_ = euler_z_sensor; }
  void set_linear_accel_x_sensor(sensor::Sensor *linear_accel_x_sensor) { linear_accel_x_sensor_ = linear_accel_x_sensor; }
  void set_linear_accel_y_sensor(sensor::Sensor *linear_accel_y_sensor) { linear_accel_y_sensor_ = linear_accel_y_sensor; }
  void set_linear_accel_z_sensor(sensor::Sensor *linear_accel_z_sensor) { linear_accel_z_sensor_ = linear_accel_z_sensor; }
  void set_gravity_x_sensor(sensor::Sensor *gravity_x_sensor) { gravity_x_sensor_ = gravity_x_sensor; }
  void set_gravity_y_sensor(sensor::Sensor *gravity_y_sensor) { gravity_y_sensor_ = gravity_y_sensor; }
  void set_gravity_z_sensor(sensor::Sensor *gravity_z_sensor) { gravity_z_sensor_ = gravity_z_sensor; }
  void set_quaternion_w_sensor(sensor::Sensor *quaternion_w_sensor) { quaternion_w_sensor_ = quaternion_w_sensor; }
  void set_quaternion_x_sensor(sensor::Sensor *quaternion_x_sensor) { quaternion_x_sensor_ = quaternion_x_sensor; }
  void set_quaternion_y_sensor(sensor::Sensor *quaternion_y_sensor) { quaternion_y_sensor_ = quaternion_y_sensor; }
  void set_quaternion_z_sensor(sensor::Sensor *quaternion_z_sensor) { quaternion_z_sensor_ = quaternion_z_sensor; }
  void set_temperature_sensor(sensor::Sensor *temperature_sensor) { temperature_sensor_ = temperature_sensor; }
  void set_magnetic_north_sensor(sensor::Sensor *magnetic_north_sensor) { magnetic_north_sensor_ = magnetic_north_sensor; }
  void set_true_heading_sensor(sensor::Sensor *true_heading_sensor) { true_heading_sensor_ = true_heading_sensor; }
  void set_magnetic_declination(float declination) { magnetic_declination_ = declination; }
  
  void set_speed_sensor(sensor::Sensor *speed_sensor) { speed_sensor_ = speed_sensor; }
  void set_distance_sensor(sensor::Sensor *distance_sensor) { distance_sensor_ = distance_sensor; }
  
  void set_calibration_system_sensor(sensor::Sensor *calibration_system_sensor) { calibration_system_sensor_ = calibration_system_sensor; }
  void set_calibration_gyro_sensor(sensor::Sensor *calibration_gyro_sensor) { calibration_gyro_sensor_ = calibration_gyro_sensor; }
  void set_calibration_accel_sensor(sensor::Sensor *calibration_accel_sensor) { calibration_accel_sensor_ = calibration_accel_sensor; }
  void set_calibration_mag_sensor(sensor::Sensor *calibration_mag_sensor) { calibration_mag_sensor_ = calibration_mag_sensor; }

 protected:
  float calculate_magnetic_north(float mag_x, float mag_y, float mag_z, float accel_x, float accel_y, float accel_z);
  float calculate_true_heading(float magnetic_heading);
  void read_calibration_status();
  void calculate_speed_distance(float linear_accel_x, float linear_accel_y, float linear_accel_z, float quat_w, float quat_x, float quat_y, float quat_z);

  uint8_t calibration_data_[22] = {0};
  void save_calibration_data();
  void restore_calibration_data();

  
  // Filtre de moyenne mobile pour l'accélération
  static const int FILTER_SIZE = 10;
  float accel_x_buffer_[FILTER_SIZE];
  float accel_y_buffer_[FILTER_SIZE];
  float accel_z_buffer_[FILTER_SIZE];
  int filter_index_{0};
  bool filter_initialized_{false};
  
  sensor::Sensor *accel_x_sensor_{nullptr};
  sensor::Sensor *accel_y_sensor_{nullptr};
  sensor::Sensor *accel_z_sensor_{nullptr};
  sensor::Sensor *gyro_x_sensor_{nullptr};
  sensor::Sensor *gyro_y_sensor_{nullptr};
  sensor::Sensor *gyro_z_sensor_{nullptr};
  sensor::Sensor *mag_x_sensor_{nullptr};
  sensor::Sensor *mag_y_sensor_{nullptr};
  sensor::Sensor *mag_z_sensor_{nullptr};
  sensor::Sensor *euler_x_sensor_{nullptr};
  sensor::Sensor *euler_y_sensor_{nullptr};
  sensor::Sensor *euler_z_sensor_{nullptr};
  sensor::Sensor *linear_accel_x_sensor_{nullptr};
  sensor::Sensor *linear_accel_y_sensor_{nullptr};
  sensor::Sensor *linear_accel_z_sensor_{nullptr};
  sensor::Sensor *gravity_x_sensor_{nullptr};
  sensor::Sensor *gravity_y_sensor_{nullptr};
  sensor::Sensor *gravity_z_sensor_{nullptr};
  sensor::Sensor *quaternion_w_sensor_{nullptr};
  sensor::Sensor *quaternion_x_sensor_{nullptr};
  sensor::Sensor *quaternion_y_sensor_{nullptr};
  sensor::Sensor *quaternion_z_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *magnetic_north_sensor_{nullptr};
  sensor::Sensor *true_heading_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *distance_sensor_{nullptr};
  sensor::Sensor *calibration_system_sensor_{nullptr};
  sensor::Sensor *calibration_gyro_sensor_{nullptr};
  sensor::Sensor *calibration_accel_sensor_{nullptr};
  sensor::Sensor *calibration_mag_sensor_{nullptr};
  float magnetic_declination_{0.0f};
  bool calibration_complete_{false};
  
  float velocity_x_{0.0f};
  float velocity_y_{0.0f};
  float velocity_z_{0.0f};
  float total_distance_{0.0f};
  uint32_t last_update_time_{0};
  bool first_update_{true};
    // For high-resolution timing
  uint32_t last_update_time_us_{0};

  // For quaternion-based gravity compensation
  float gravity_x_{0.0f};
  float gravity_y_{0.0f};
  float gravity_z_{0.0f};
};

}  // namespace bno055
}  // namespace esphome 
