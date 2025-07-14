#include "bno055.h"
#include "esphome/core/log.h"
#include <cmath>

namespace esphome {
namespace bno055 {

static const char *const TAG = "bno055";

const uint8_t BNO055_REGISTER_CHIP_ID = 0x00;
const uint8_t BNO0555_REGISTER_ACCEL_ID = 0x01;
const uint8_t BNO055_REGISTER_MAG_ID = 0x02;
const uint8_t BNO055_REGISTER_GYRO_ID = 0x03;
const uint8_t BNO055_REGISTER_SW_REV_ID_LSB = 0x04;
const uint8_t BNO055_REGISTER_SW_REV_ID_MSB = 0x05;
const uint8_t BNO055_REGISTER_BL_REV_ID = 0x06;
const uint8_t BNO055_REGISTER_PAGE_ID = 0x07;
const uint8_t BNO055_REGISTER_ACCEL_DATA_X_LSB = 0x08;
const uint8_t BNO055_REGISTER_ACCEL_DATA_X_MSB = 0x09;
const uint8_t BNO055_REGISTER_ACCEL_DATA_Y_LSB = 0x0A;
const uint8_t BNO055_REGISTER_ACCEL_DATA_Y_MSB = 0x0B;
const uint8_t BNO055_REGISTER_ACCEL_DATA_Z_LSB = 0x0C;
const uint8_t BNO055_REGISTER_ACCEL_DATA_Z_MSB = 0x0D;
const uint8_t BNO055_REGISTER_MAG_DATA_X_LSB = 0x0E;
const uint8_t BNO055_REGISTER_MAG_DATA_X_MSB = 0x0F;
const uint8_t BNO055_REGISTER_MAG_DATA_Y_LSB = 0x10;
const uint8_t BNO055_REGISTER_MAG_DATA_Y_MSB = 0x11;
const uint8_t BNO055_REGISTER_MAG_DATA_Z_LSB = 0x12;
const uint8_t BNO055_REGISTER_MAG_DATA_Z_MSB = 0x13;
const uint8_t BNO055_REGISTER_GYRO_DATA_X_LSB = 0x14;
const uint8_t BNO055_REGISTER_GYRO_DATA_X_MSB = 0x15;
const uint8_t BNO055_REGISTER_GYRO_DATA_Y_LSB = 0x16;
const uint8_t BNO055_REGISTER_GYRO_DATA_Y_MSB = 0x17;
const uint8_t BNO055_REGISTER_GYRO_DATA_Z_LSB = 0x18;
const uint8_t BNO055_REGISTER_GYRO_DATA_Z_MSB = 0x19;
const uint8_t BNO055_REGISTER_EULER_H_LSB = 0x1A;
const uint8_t BNO055_REGISTER_EULER_H_MSB = 0x1B;
const uint8_t BNO055_REGISTER_EULER_R_LSB = 0x1C;
const uint8_t BNO055_REGISTER_EULER_R_MSB = 0x1D;
const uint8_t BNO055_REGISTER_EULER_P_LSB = 0x1E;
const uint8_t BNO055_REGISTER_EULER_P_MSB = 0x1F;
const uint8_t BNO055_REGISTER_QUATERNION_DATA_W_LSB = 0x20;
const uint8_t BNO055_REGISTER_QUATERNION_DATA_W_MSB = 0x21;
const uint8_t BNO055_REGISTER_QUATERNION_DATA_X_LSB = 0x22;
const uint8_t BNO055_REGISTER_QUATERNION_DATA_X_MSB = 0x23;
const uint8_t BNO055_REGISTER_QUATERNION_DATA_Y_LSB = 0x24;
const uint8_t BNO055_REGISTER_QUATERNION_DATA_Y_MSB = 0x25;
const uint8_t BNO055_REGISTER_QUATERNION_DATA_Z_LSB = 0x26;
const uint8_t BNO055_REGISTER_QUATERNION_DATA_Z_MSB = 0x27;
const uint8_t BNO055_REGISTER_LINEAR_ACCEL_DATA_X_LSB = 0x28;
const uint8_t BNO055_REGISTER_LINEAR_ACCEL_DATA_X_MSB = 0x29;
const uint8_t BNO055_REGISTER_LINEAR_ACCEL_DATA_Y_LSB = 0x2A;
const uint8_t BNO055_REGISTER_LINEAR_ACCEL_DATA_Y_MSB = 0x2B;
const uint8_t BNO055_REGISTER_LINEAR_ACCEL_DATA_Z_LSB = 0x2C;
const uint8_t BNO055_REGISTER_LINEAR_ACCEL_DATA_Z_MSB = 0x2D;
const uint8_t BNO055_REGISTER_GRAVITY_DATA_X_LSB = 0x2E;
const uint8_t BNO055_REGISTER_GRAVITY_DATA_X_MSB = 0x2F;
const uint8_t BNO055_REGISTER_GRAVITY_DATA_Y_LSB = 0x30;
const uint8_t BNO055_REGISTER_GRAVITY_DATA_Y_MSB = 0x31;
const uint8_t BNO055_REGISTER_GRAVITY_DATA_Z_LSB = 0x32;
const uint8_t BNO055_REGISTER_GRAVITY_DATA_Z_MSB = 0x33;
const uint8_t BNO055_REGISTER_TEMP = 0x34;
const uint8_t BNO055_REGISTER_CALIB_STAT = 0x35;
const uint8_t BNO055_REGISTER_ST_RESULT = 0x36;
const uint8_t BNO055_REGISTER_INT_STA = 0x37;
const uint8_t BNO055_REGISTER_SYS_CLK_STAT = 0x38;
const uint8_t BNO055_REGISTER_SYS_STAT = 0x39;
const uint8_t BNO055_REGISTER_SYS_ERR = 0x3A;
const uint8_t BNO055_REGISTER_UNIT_SEL = 0x3B;
const uint8_t BNO055_REGISTER_OPR_MODE = 0x3D;
const uint8_t BNO055_REGISTER_PWR_MODE = 0x3E;
const uint8_t BNO055_REGISTER_SYS_TRIGGER = 0x3F;
const uint8_t BNO055_REGISTER_TEMP_SOURCE = 0x40;
const uint8_t BNO055_REGISTER_AXIS_MAP_CONFIG = 0x41;
const uint8_t BNO055_REGISTER_AXIS_MAP_SIGN = 0x42;
const uint8_t BNO055_REGISTER_CALIB_DATA = 0x55;
const uint8_t BNO055_REGISTER_CALIB_OFFSET_ACCEL = 0x55;
const uint8_t BNO055_REGISTER_CALIB_OFFSET_MAG = 0x5B;
const uint8_t BNO055_REGISTER_CALIB_OFFSET_GYRO = 0x61;
const uint8_t BNO055_REGISTER_CALIB_RADIUS_ACCEL = 0x67;
const uint8_t BNO055_REGISTER_CALIB_RADIUS_MAG = 0x69;

const uint8_t BNO055_CHIP_ID = 0xA0;
const uint8_t BNO055_ACCEL_ID = 0xFB;
const uint8_t BNO055_MAG_ID = 0x32;
const uint8_t BNO055_GYRO_ID = 0x0F;

const uint8_t BNO055_OPR_MODE_CONFIG = 0x00;
const uint8_t BNO055_OPR_MODE_ACCONLY = 0x01;
const uint8_t BNO055_OPR_MODE_MAGONLY = 0x02;
const uint8_t BNO055_OPR_MODE_GYROONLY = 0x03;
const uint8_t BNO055_OPR_MODE_ACCMAG = 0x04;
const uint8_t BNO055_OPR_MODE_ACCGYRO = 0x05;
const uint8_t BNO055_OPR_MODE_MAGGYRO = 0x06;
const uint8_t BNO055_OPR_MODE_AMG = 0x07;
const uint8_t BNO055_OPR_MODE_IMU = 0x08;
const uint8_t BNO055_OPR_MODE_COMPASS = 0x09;
const uint8_t BNO055_OPR_MODE_M4G = 0x0A;
const uint8_t BNO055_OPR_MODE_NDOF_FMC_OFF = 0x0B;
const uint8_t BNO055_OPR_MODE_NDOF = 0x0C;

const uint8_t BNO055_PWR_MODE_NORMAL = 0x00;
const uint8_t BNO055_PWR_MODE_LOWPOWER = 0x01;
const uint8_t BNO055_PWR_MODE_SUSPEND = 0x02;

const uint8_t BNO055_UNIT_ACCEL_MS2 = 0x00;
const uint8_t BNO055_UNIT_ACCEL_MG = 0x01;
const uint8_t BNO055_UNIT_GYRO_DPS = 0x00;
const uint8_t BNO055_UNIT_GYRO_RPS = 0x02;
const uint8_t BNO055_UNIT_EULER_DEG = 0x00;
const uint8_t BNO055_UNIT_EULER_RAD = 0x04;
const uint8_t BNO055_UNIT_TEMP_C = 0x00;
const uint8_t BNO055_UNIT_TEMP_F = 0x10;
const uint8_t BNO055_UNIT_ORI_ANDROID = 0x00;
const uint8_t BNO055_UNIT_ORI_WINDOWS = 0x80;

void BNO055Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BNO055...");
  
  uint8_t chip_id;
  if (!this->read_byte(BNO055_REGISTER_CHIP_ID, &chip_id) || chip_id != BNO055_CHIP_ID) {
    ESP_LOGE(TAG, "BNO055 not found at address 0x%02X", this->address_);
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "BNO055 found with chip ID: 0x%02X", chip_id);
  
  uint8_t accel_id, mag_id, gyro_id;
  if (!this->read_byte(BNO055_REGISTER_ACCEL_ID, &accel_id) ||
      !this->read_byte(BNO055_REGISTER_MAG_ID, &mag_id) ||
      !this->read_byte(BNO055_REGISTER_GYRO_ID, &gyro_id)) {
    ESP_LOGE(TAG, "Failed to read sensor IDs");
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "Sensor IDs - Accel: 0x%02X, Mag: 0x%02X, Gyro: 0x%02X", accel_id, mag_id, gyro_id);
  
  if (accel_id != BNO055_ACCEL_ID || mag_id != BNO055_MAG_ID || gyro_id != BNO055_GYRO_ID) {
    ESP_LOGE(TAG, "Invalid sensor IDs");
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "Setting operation mode to NDOF");
  if (!this->write_byte(BNO055_REGISTER_OPR_MODE, BNO055_OPR_MODE_NDOF)) {
    ESP_LOGE(TAG, "Failed to set operation mode");
    this->mark_failed();
    return;
  }
  
  delay(30);
  
  ESP_LOGCONFIG(TAG, "Setting power mode to normal");
  if (!this->write_byte(BNO055_REGISTER_PWR_MODE, BNO055_PWR_MODE_NORMAL)) {
    ESP_LOGE(TAG, "Failed to set power mode");
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "Setting units");
  uint8_t unit_sel = BNO055_UNIT_ACCEL_MS2 | BNO055_UNIT_GYRO_DPS | BNO055_UNIT_EULER_DEG | BNO055_UNIT_TEMP_C | BNO055_UNIT_ORI_ANDROID;
  if (!this->write_byte(BNO055_REGISTER_UNIT_SEL, unit_sel)) {
    ESP_LOGE(TAG, "Failed to set units");
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "BNO055 setup complete");
}

void BNO055Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BNO055:");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication failed!");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Acceleration X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Acceleration Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Acceleration Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Magnetic X", this->mag_x_sensor_);
  LOG_SENSOR("  ", "Magnetic Y", this->mag_y_sensor_);
  LOG_SENSOR("  ", "Magnetic Z", this->mag_z_sensor_);
  LOG_SENSOR("  ", "Euler X", this->euler_x_sensor_);
  LOG_SENSOR("  ", "Euler Y", this->euler_y_sensor_);
  LOG_SENSOR("  ", "Euler Z", this->euler_z_sensor_);
  LOG_SENSOR("  ", "Linear Acceleration X", this->linear_accel_x_sensor_);
  LOG_SENSOR("  ", "Linear Acceleration Y", this->linear_accel_y_sensor_);
  LOG_SENSOR("  ", "Linear Acceleration Z", this->linear_accel_z_sensor_);
  LOG_SENSOR("  ", "Gravity X", this->gravity_x_sensor_);
  LOG_SENSOR("  ", "Gravity Y", this->gravity_y_sensor_);
  LOG_SENSOR("  ", "Gravity Z", this->gravity_z_sensor_);
  LOG_SENSOR("  ", "Quaternion W", this->quaternion_w_sensor_);
  LOG_SENSOR("  ", "Quaternion X", this->quaternion_x_sensor_);
  LOG_SENSOR("  ", "Quaternion Y", this->quaternion_y_sensor_);
  LOG_SENSOR("  ", "Quaternion Z", this->quaternion_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Magnetic North", this->magnetic_north_sensor_);
  LOG_SENSOR("  ", "True Heading", this->true_heading_sensor_);
  LOG_SENSOR("  ", "Speed", this->speed_sensor_);
  LOG_SENSOR("  ", "Distance", this->distance_sensor_);
  LOG_SENSOR("  ", "Calibration System", this->calibration_system_sensor_);
  LOG_SENSOR("  ", "Calibration Gyro", this->calibration_gyro_sensor_);
  LOG_SENSOR("  ", "Calibration Accel", this->calibration_accel_sensor_);
  LOG_SENSOR("  ", "Calibration Mag", this->calibration_mag_sensor_);
}

float BNO055Component::calculate_magnetic_north(float mag_x, float mag_y, float mag_z, float accel_x, float accel_y, float accel_z) {
  float roll = atan2(accel_y, accel_z);
  float pitch = atan2(-accel_x, sqrt(accel_y * accel_y + accel_z * accel_z));
  
  float cos_roll = cos(roll);
  float sin_roll = sin(roll);
  float cos_pitch = cos(pitch);
  float sin_pitch = sin(pitch);
  
  // Correct transformation for BNO055 coordinate system
  float mag_x_comp = mag_x * cos_pitch - mag_z * sin_pitch;
  float mag_y_comp = mag_x * sin_roll * sin_pitch + mag_y * cos_roll + mag_z * sin_roll * cos_pitch;
  
  float heading = atan2(mag_y_comp, mag_x_comp);
  
  if (heading < 0) {
    heading += 2 * M_PI;
  }
  
  return heading * 180.0f / M_PI;
}

float BNO055Component::calculate_true_heading(float magnetic_heading) {
  float true_heading = magnetic_heading + magnetic_declination_;
  
  if (true_heading >= 360.0f) {
    true_heading -= 360.0f;
  } else if (true_heading < 0.0f) {
    true_heading += 360.0f;
  }
  
  return true_heading;
}

void BNO055Component::read_calibration_status() {
  uint8_t calib_status;
  if (!this->read_byte(BNO055_REGISTER_CALIB_STAT, &calib_status)) {
    return;
  }
  
  uint8_t system_calib = (calib_status >> 6) & 0x03;
  uint8_t gyro_calib = (calib_status >> 4) & 0x03;
  uint8_t accel_calib = (calib_status >> 2) & 0x03;
  uint8_t mag_calib = calib_status & 0x03;
  
  if (this->calibration_system_sensor_ != nullptr)
    this->calibration_system_sensor_->publish_state(system_calib);
  if (this->calibration_gyro_sensor_ != nullptr)
    this->calibration_gyro_sensor_->publish_state(gyro_calib);
  if (this->calibration_accel_sensor_ != nullptr)
    this->calibration_accel_sensor_->publish_state(accel_calib);
  if (this->calibration_mag_sensor_ != nullptr)
    this->calibration_mag_sensor_->publish_state(mag_calib);
  
  bool was_complete = calibration_complete_;
  calibration_complete_ = (system_calib == 3) && (gyro_calib == 3) && (accel_calib == 3) && (mag_calib == 3);
  
  // Publish calibration complete binary sensor
  if (this->calibration_complete_binary_sensor_ != nullptr) {
    this->calibration_complete_binary_sensor_->publish_state(calibration_complete_);
  }
  
  if (calibration_complete_ && !was_complete) {
    ESP_LOGI(TAG, "BNO055 calibration complete!");
  } else if (!calibration_complete_ && was_complete) {
    ESP_LOGW(TAG, "BNO055 calibration lost!");
  }
  
  if (!calibration_complete_) {
    ESP_LOGD(TAG, "Calibration status - System: %d, Gyro: %d, Accel: %d, Mag: %d", 
             system_calib, gyro_calib, accel_calib, mag_calib);
  }
}

void BNO055Component::calculate_speed_distance(float linear_accel_x, float linear_accel_y, float linear_accel_z) {
  uint32_t current_time = millis();
  float dt = 0.0f;
  
  if (!first_update_) {
    dt = (current_time - last_update_time_) / 1000.0f;
  } else {
    first_update_ = false;
  }
  
  last_update_time_ = current_time;
  
  if (dt > 0.0f && dt < 1.0f) {
    velocity_x_ += linear_accel_x * dt;
    velocity_y_ += linear_accel_y * dt;
    velocity_z_ += linear_accel_z * dt;
    
    float speed = sqrt(velocity_x_ * velocity_x_ + velocity_y_ * velocity_y_ + velocity_z_ * velocity_z_);
    float distance_increment = speed * dt;
    total_distance_ += distance_increment;
    
    if (this->speed_sensor_ != nullptr)
      this->speed_sensor_->publish_state(speed);
    if (this->distance_sensor_ != nullptr)
      this->distance_sensor_->publish_state(total_distance_);
  }
}

void BNO055Component::update() {
  ESP_LOGV(TAG, "Updating BNO055 sensors");
  
  read_calibration_status();
  
  uint8_t data[22];
  if (!this->read_bytes(BNO055_REGISTER_ACCEL_DATA_X_LSB, data, 22)) {
    this->status_set_warning();
    return;
  }
  
  int16_t accel_x = (data[1] << 8) | data[0];
  int16_t accel_y = (data[3] << 8) | data[2];
  int16_t accel_z = (data[5] << 8) | data[4];
  int16_t mag_x = (data[7] << 8) | data[6];
  int16_t mag_y = (data[9] << 8) | data[8];
  int16_t mag_z = (data[11] << 8) | data[10];
  int16_t gyro_x = (data[13] << 8) | data[12];
  int16_t gyro_y = (data[15] << 8) | data[14];
  int16_t gyro_z = (data[17] << 8) | data[16];
  int16_t euler_h = (data[19] << 8) | data[18];
  int16_t euler_r = (data[21] << 8) | data[20];
  
  if (!this->read_bytes(BNO055_REGISTER_EULER_P_LSB, data, 2)) {
    this->status_set_warning();
    return;
  }
  int16_t euler_p = (data[1] << 8) | data[0];
  
  if (!this->read_bytes(BNO055_REGISTER_QUATERNION_DATA_W_LSB, data, 8)) {
    this->status_set_warning();
    return;
  }
  int16_t quat_w = (data[1] << 8) | data[0];
  int16_t quat_x = (data[3] << 8) | data[2];
  int16_t quat_y = (data[5] << 8) | data[4];
  int16_t quat_z = (data[7] << 8) | data[6];
  
  if (!this->read_bytes(BNO055_REGISTER_LINEAR_ACCEL_DATA_X_LSB, data, 6)) {
    this->status_set_warning();
    return;
  }
  int16_t linear_accel_x = (data[1] << 8) | data[0];
  int16_t linear_accel_y = (data[3] << 8) | data[2];
  int16_t linear_accel_z = (data[5] << 8) | data[4];
  
  if (!this->read_bytes(BNO055_REGISTER_GRAVITY_DATA_X_LSB, data, 6)) {
    this->status_set_warning();
    return;
  }
  int16_t gravity_x = (data[1] << 8) | data[0];
  int16_t gravity_y = (data[3] << 8) | data[2];
  int16_t gravity_z = (data[5] << 8) | data[4];
  
  uint8_t temp;
  if (!this->read_byte(BNO055_REGISTER_TEMP, &temp)) {
    this->status_set_warning();
    return;
  }
  
  float accel_x_f = accel_x / 100.0f;
  float accel_y_f = accel_y / 100.0f;
  float accel_z_f = accel_z / 100.0f;
  float gyro_x_f = gyro_x / 16.0f;
  float gyro_y_f = gyro_y / 16.0f;
  float gyro_z_f = gyro_z / 16.0f;
  float mag_x_f = mag_x / 16.0f;
  float mag_y_f = mag_y / 16.0f;
  float mag_z_f = mag_z / 16.0f;
  float euler_h_f = euler_h / 16.0f;
  float euler_r_f = euler_r / 16.0f;
  float euler_p_f = euler_p / 16.0f;
  float quat_w_f = quat_w / 16384.0f;
  float quat_x_f = quat_x / 16384.0f;
  float quat_y_f = quat_y / 16384.0f;
  float quat_z_f = quat_z / 16384.0f;
  float linear_accel_x_f = linear_accel_x / 100.0f;
  float linear_accel_y_f = linear_accel_y / 100.0f;
  float linear_accel_z_f = linear_accel_z / 100.0f;
  float gravity_x_f = gravity_x / 100.0f;
  float gravity_y_f = gravity_y / 100.0f;
  float gravity_z_f = gravity_z / 100.0f;
  float temp_f = temp;
  
  calculate_speed_distance(linear_accel_x_f, linear_accel_y_f, linear_accel_z_f);
  
  float magnetic_north = euler_h_f;
  float true_heading = calculate_true_heading(magnetic_north);
  
  ESP_LOGD(TAG,
           "Got accel={x=%.3f m/s², y=%.3f m/s², z=%.3f m/s²}, "
           "gyro={x=%.3f °/s, y=%.3f °/s, z=%.3f °/s}, "
           "mag={x=%.3f μT, y=%.3f μT, z=%.3f μT}, "
           "euler={h=%.3f°, r=%.3f°, p=%.3f°}, "
           "magnetic_north=%.1f°, temp=%.1f°C",
           accel_x_f, accel_y_f, accel_z_f, gyro_x_f, gyro_y_f, gyro_z_f,
           mag_x_f, mag_y_f, mag_z_f, euler_h_f, euler_r_f, euler_p_f, magnetic_north, temp_f);
  
  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(accel_x_f);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(accel_y_f);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(accel_z_f);
  
  if (this->gyro_x_sensor_ != nullptr)
    this->gyro_x_sensor_->publish_state(gyro_x_f);
  if (this->gyro_y_sensor_ != nullptr)
    this->gyro_y_sensor_->publish_state(gyro_y_f);
  if (this->gyro_z_sensor_ != nullptr)
    this->gyro_z_sensor_->publish_state(gyro_z_f);
  
  if (this->mag_x_sensor_ != nullptr)
    this->mag_x_sensor_->publish_state(mag_x_f);
  if (this->mag_y_sensor_ != nullptr)
    this->mag_y_sensor_->publish_state(mag_y_f);
  if (this->mag_z_sensor_ != nullptr)
    this->mag_z_sensor_->publish_state(mag_z_f);
  
  if (this->euler_x_sensor_ != nullptr)
    this->euler_x_sensor_->publish_state(euler_h_f);
  if (this->euler_y_sensor_ != nullptr)
    this->euler_y_sensor_->publish_state(euler_r_f);
  if (this->euler_z_sensor_ != nullptr)
    this->euler_z_sensor_->publish_state(euler_p_f);
  
  if (this->linear_accel_x_sensor_ != nullptr)
    this->linear_accel_x_sensor_->publish_state(linear_accel_x_f);
  if (this->linear_accel_y_sensor_ != nullptr)
    this->linear_accel_y_sensor_->publish_state(linear_accel_y_f);
  if (this->linear_accel_z_sensor_ != nullptr)
    this->linear_accel_z_sensor_->publish_state(linear_accel_z_f);
  
  if (this->gravity_x_sensor_ != nullptr)
    this->gravity_x_sensor_->publish_state(gravity_x_f);
  if (this->gravity_y_sensor_ != nullptr)
    this->gravity_y_sensor_->publish_state(gravity_y_f);
  if (this->gravity_z_sensor_ != nullptr)
    this->gravity_z_sensor_->publish_state(gravity_z_f);
  
  if (this->quaternion_w_sensor_ != nullptr)
    this->quaternion_w_sensor_->publish_state(quat_w_f);
  if (this->quaternion_x_sensor_ != nullptr)
    this->quaternion_x_sensor_->publish_state(quat_x_f);
  if (this->quaternion_y_sensor_ != nullptr)
    this->quaternion_y_sensor_->publish_state(quat_y_f);
  if (this->quaternion_z_sensor_ != nullptr)
    this->quaternion_z_sensor_->publish_state(quat_z_f);
  
  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temp_f);
  
  if (this->magnetic_north_sensor_ != nullptr)
    this->magnetic_north_sensor_->publish_state(magnetic_north);
  
  if (this->true_heading_sensor_ != nullptr)
    this->true_heading_sensor_->publish_state(true_heading);
  
  this->status_clear_warning();
}

float BNO055Component::get_setup_priority() const { return setup_priority::DATA; }

}  // namespace bno055
}  // namespace esphome 
