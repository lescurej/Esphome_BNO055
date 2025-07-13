#include "bno055.h"
#include "esphome/core/log.h"

namespace esphome {
namespace bno055 {

static const char *const TAG = "bno055";

// BNO055 Register addresses
static const uint8_t BNO055_CHIP_ID_ADDR = 0x00;
static const uint8_t BNO055_ACCEL_REV_ID_ADDR = 0x01;
static const uint8_t BNO055_MAG_REV_ID_ADDR = 0x02;
static const uint8_t BNO055_GYRO_REV_ID_ADDR = 0x03;
static const uint8_t BNO055_SW_REV_ID_LSB_ADDR = 0x04;
static const uint8_t BNO055_SW_REV_ID_MSB_ADDR = 0x05;
static const uint8_t BNO055_BL_REV_ID_ADDR = 0x06;
static const uint8_t BNO055_PAGE_ID_ADDR = 0x07;
static const uint8_t BNO055_ACCEL_DATA_X_LSB_ADDR = 0x08;
static const uint8_t BNO055_ACCEL_DATA_X_MSB_ADDR = 0x09;
static const uint8_t BNO055_ACCEL_DATA_Y_LSB_ADDR = 0x0A;
static const uint8_t BNO055_ACCEL_DATA_Y_MSB_ADDR = 0x0B;
static const uint8_t BNO055_ACCEL_DATA_Z_LSB_ADDR = 0x0C;
static const uint8_t BNO055_ACCEL_DATA_Z_MSB_ADDR = 0x0D;
static const uint8_t BNO055_MAG_DATA_X_LSB_ADDR = 0x0E;
static const uint8_t BNO055_MAG_DATA_X_MSB_ADDR = 0x0F;
static const uint8_t BNO055_MAG_DATA_Y_LSB_ADDR = 0x10;
static const uint8_t BNO055_MAG_DATA_Y_MSB_ADDR = 0x11;
static const uint8_t BNO055_MAG_DATA_Z_LSB_ADDR = 0x12;
static const uint8_t BNO055_MAG_DATA_Z_MSB_ADDR = 0x13;
static const uint8_t BNO055_GYRO_DATA_X_LSB_ADDR = 0x14;
static const uint8_t BNO055_GYRO_DATA_X_MSB_ADDR = 0x15;
static const uint8_t BNO055_GYRO_DATA_Y_LSB_ADDR = 0x16;
static const uint8_t BNO055_GYRO_DATA_Y_MSB_ADDR = 0x17;
static const uint8_t BNO055_GYRO_DATA_Z_LSB_ADDR = 0x18;
static const uint8_t BNO055_GYRO_DATA_Z_MSB_ADDR = 0x19;
static const uint8_t BNO055_EULER_H_LSB_ADDR = 0x1A;
static const uint8_t BNO055_EULER_H_MSB_ADDR = 0x1B;
static const uint8_t BNO055_EULER_R_LSB_ADDR = 0x1C;
static const uint8_t BNO055_EULER_R_MSB_ADDR = 0x1D;
static const uint8_t BNO055_EULER_P_LSB_ADDR = 0x1E;
static const uint8_t BNO055_EULER_P_MSB_ADDR = 0x1F;
static const uint8_t BNO055_QUATERNION_DATA_W_LSB_ADDR = 0x20;
static const uint8_t BNO055_QUATERNION_DATA_W_MSB_ADDR = 0x21;
static const uint8_t BNO055_QUATERNION_DATA_X_LSB_ADDR = 0x22;
static const uint8_t BNO055_QUATERNION_DATA_X_MSB_ADDR = 0x23;
static const uint8_t BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0x24;
static const uint8_t BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0x25;
static const uint8_t BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0x26;
static const uint8_t BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0x27;
static const uint8_t BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0x28;
static const uint8_t BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0x29;
static const uint8_t BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0x2A;
static const uint8_t BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0x2B;
static const uint8_t BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0x2C;
static const uint8_t BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0x2D;
static const uint8_t BNO055_GRAVITY_DATA_X_LSB_ADDR = 0x2E;
static const uint8_t BNO055_GRAVITY_DATA_X_MSB_ADDR = 0x2F;
static const uint8_t BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0x30;
static const uint8_t BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0x31;
static const uint8_t BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0x32;
static const uint8_t BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0x33;
static const uint8_t BNO055_TEMP_ADDR = 0x34;
static const uint8_t BNO055_CALIB_STAT_ADDR = 0x35;
static const uint8_t BNO055_SELFTEST_RESULT_ADDR = 0x36;
static const uint8_t BNO055_INTR_STAT_ADDR = 0x37;
static const uint8_t BNO055_SYS_CLK_STAT_ADDR = 0x38;
static const uint8_t BNO055_SYS_STAT_ADDR = 0x39;
static const uint8_t BNO055_SYS_ERR_ADDR = 0x3A;
static const uint8_t BNO055_UNIT_SEL_ADDR = 0x3B;
static const uint8_t BNO055_DATA_SEL_ADDR = 0x3C;
static const uint8_t BNO055_OPR_MODE_ADDR = 0x3D;
static const uint8_t BNO055_PWR_MODE_ADDR = 0x3E;
static const uint8_t BNO055_SYS_TRIGGER_ADDR = 0x3F;
static const uint8_t BNO055_TEMP_SOURCE_ADDR = 0x40;
static const uint8_t BNO055_AXIS_MAP_CONFIG_ADDR = 0x41;
static const uint8_t BNO055_AXIS_MAP_SIGN_ADDR = 0x42;
static const uint8_t BNO055_SIC_MATRIX_0_LSB_ADDR = 0x43;
static const uint8_t BNO055_SIC_MATRIX_0_MSB_ADDR = 0x44;
static const uint8_t BNO055_SIC_MATRIX_1_LSB_ADDR = 0x45;
static const uint8_t BNO055_SIC_MATRIX_1_MSB_ADDR = 0x46;
static const uint8_t BNO055_SIC_MATRIX_2_LSB_ADDR = 0x47;
static const uint8_t BNO055_SIC_MATRIX_2_MSB_ADDR = 0x48;
static const uint8_t BNO055_SIC_MATRIX_3_LSB_ADDR = 0x49;
static const uint8_t BNO055_SIC_MATRIX_3_MSB_ADDR = 0x4A;
static const uint8_t BNO055_SIC_MATRIX_4_LSB_ADDR = 0x4B;
static const uint8_t BNO055_SIC_MATRIX_4_MSB_ADDR = 0x4C;
static const uint8_t BNO055_SIC_MATRIX_5_LSB_ADDR = 0x4D;
static const uint8_t BNO055_SIC_MATRIX_5_MSB_ADDR = 0x4E;
static const uint8_t BNO055_SIC_MATRIX_6_LSB_ADDR = 0x4F;
static const uint8_t BNO055_SIC_MATRIX_6_MSB_ADDR = 0x50;
static const uint8_t BNO055_SIC_MATRIX_7_LSB_ADDR = 0x51;
static const uint8_t BNO055_SIC_MATRIX_7_MSB_ADDR = 0x52;
static const uint8_t BNO055_SIC_MATRIX_8_LSB_ADDR = 0x53;
static const uint8_t BNO055_SIC_MATRIX_8_MSB_ADDR = 0x54;
static const uint8_t BNO055_ACCEL_OFFSET_X_LSB_ADDR = 0x55;
static const uint8_t BNO055_ACCEL_OFFSET_X_MSB_ADDR = 0x56;
static const uint8_t BNO055_ACCEL_OFFSET_Y_LSB_ADDR = 0x57;
static const uint8_t BNO055_ACCEL_OFFSET_Y_MSB_ADDR = 0x58;
static const uint8_t BNO055_ACCEL_OFFSET_Z_LSB_ADDR = 0x59;
static const uint8_t BNO055_ACCEL_OFFSET_Z_MSB_ADDR = 0x5A;
static const uint8_t BNO055_MAG_OFFSET_X_LSB_ADDR = 0x5B;
static const uint8_t BNO055_MAG_OFFSET_X_MSB_ADDR = 0x5C;
static const uint8_t BNO055_MAG_OFFSET_Y_LSB_ADDR = 0x5D;
static const uint8_t BNO055_MAG_OFFSET_Y_MSB_ADDR = 0x5E;
static const uint8_t BNO055_MAG_OFFSET_Z_LSB_ADDR = 0x5F;
static const uint8_t BNO055_MAG_OFFSET_Z_MSB_ADDR = 0x60;
static const uint8_t BNO055_GYRO_OFFSET_X_LSB_ADDR = 0x61;
static const uint8_t BNO055_GYRO_OFFSET_X_MSB_ADDR = 0x62;
static const uint8_t BNO055_GYRO_OFFSET_Y_LSB_ADDR = 0x63;
static const uint8_t BNO055_GYRO_OFFSET_Y_MSB_ADDR = 0x64;
static const uint8_t BNO055_GYRO_OFFSET_Z_LSB_ADDR = 0x65;
static const uint8_t BNO055_GYRO_OFFSET_Z_MSB_ADDR = 0x66;
static const uint8_t BNO055_ACCEL_RADIUS_MSB_ADDR = 0x67;
static const uint8_t BNO055_ACCEL_RADIUS_LSB_ADDR = 0x68;
static const uint8_t BNO055_MAG_RADIUS_MSB_ADDR = 0x69;
static const uint8_t BNO055_MAG_RADIUS_LSB_ADDR = 0x6A;

// BNO055 Chip ID
static const uint8_t BNO055_ID = 0xA0;

// BNO055 Operating modes
static const uint8_t BNO055_OPERATION_MODE_CONFIG = 0x00;
static const uint8_t BNO055_OPERATION_MODE_ACCONLY = 0x01;
static const uint8_t BNO055_OPERATION_MODE_MAGONLY = 0x02;
static const uint8_t BNO055_OPERATION_MODE_GYROONLY = 0x03;
static const uint8_t BNO055_OPERATION_MODE_ACCMAG = 0x04;
static const uint8_t BNO055_OPERATION_MODE_ACCGYRO = 0x05;
static const uint8_t BNO055_OPERATION_MODE_MAGGYRO = 0x06;
static const uint8_t BNO055_OPERATION_MODE_AMG = 0x07;
static const uint8_t BNO055_OPERATION_MODE_IMU = 0x08;
static const uint8_t BNO055_OPERATION_MODE_COMPASS = 0x09;
static const uint8_t BNO055_OPERATION_MODE_M4G = 0x0A;
static const uint8_t BNO055_OPERATION_MODE_NDOF_FMC_OFF = 0x0B;
static const uint8_t BNO055_OPERATION_MODE_NDOF = 0x0C;

void BNO055Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BNO055...");
  
  uint8_t chip_id;
  if (!this->read_byte_(BNO055_CHIP_ID_ADDR, &chip_id)) {
    this->error_code_ = COMMUNICATION_FAILED;
    this->mark_failed();
    return;
  }
  
  if (chip_id != BNO055_ID) {
    ESP_LOGE(TAG, "BNO055 not found. Expected ID 0x%02X, got 0x%02X", BNO055_ID, chip_id);
    this->error_code_ = ID_MISMATCH;
    this->mark_failed();
    return;
  }
  
  // Reset the device
  if (!this->write_byte_(BNO055_SYS_TRIGGER_ADDR, 0x20)) {
    ESP_LOGE(TAG, "Failed to reset BNO055");
    this->error_code_ = INITIALIZATION_FAILED;
    this->mark_failed();
    return;
  }
  
  delay(650);
  
  // Set to config mode
  if (!this->write_byte_(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG)) {
    ESP_LOGE(TAG, "Failed to set config mode");
    this->error_code_ = INITIALIZATION_FAILED;
    this->mark_failed();
    return;
  }
  
  delay(25);
  
  // Set to NDOF mode (optimized for marine use with tilt compensation)
  if (!this->write_byte_(BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF)) {
    ESP_LOGE(TAG, "Failed to set NDOF mode");
    this->error_code_ = INITIALIZATION_FAILED;
    this->mark_failed();
    return;
  }
  
  delay(25);
  
  ESP_LOGCONFIG(TAG, "BNO055 setup complete");
}

void BNO055Component::dump_config() {
  ESP_LOGCONFIG(TAG, "BNO055:");
  LOG_I2C_DEVICE(this);
  
  if (this->error_code_ == COMMUNICATION_FAILED) {
    ESP_LOGE(TAG, "Communication failed");
  } else if (this->error_code_ == ID_MISMATCH) {
    ESP_LOGE(TAG, "ID mismatch");
  } else if (this->error_code_ == INITIALIZATION_FAILED) {
    ESP_LOGE(TAG, "Initialization failed");
  }
  
  LOG_SENSOR("  ", "Accel X", this->accel_x_sensor_);
  LOG_SENSOR("  ", "Accel Y", this->accel_y_sensor_);
  LOG_SENSOR("  ", "Accel Z", this->accel_z_sensor_);
  LOG_SENSOR("  ", "Gyro X", this->gyro_x_sensor_);
  LOG_SENSOR("  ", "Gyro Y", this->gyro_y_sensor_);
  LOG_SENSOR("  ", "Gyro Z", this->gyro_z_sensor_);
  LOG_SENSOR("  ", "Mag X", this->mag_x_sensor_);
  LOG_SENSOR("  ", "Mag Y", this->mag_y_sensor_);
  LOG_SENSOR("  ", "Mag Z", this->mag_z_sensor_);
  LOG_SENSOR("  ", "Euler H", this->euler_h_sensor_);
  LOG_SENSOR("  ", "Euler R", this->euler_r_sensor_);
  LOG_SENSOR("  ", "Euler P", this->euler_p_sensor_);
  LOG_SENSOR("  ", "Linear Accel X", this->linear_accel_x_sensor_);
  LOG_SENSOR("  ", "Linear Accel Y", this->linear_accel_y_sensor_);
  LOG_SENSOR("  ", "Linear Accel Z", this->linear_accel_z_sensor_);
  LOG_SENSOR("  ", "Gravity X", this->gravity_x_sensor_);
  LOG_SENSOR("  ", "Gravity Y", this->gravity_y_sensor_);
  LOG_SENSOR("  ", "Gravity Z", this->gravity_z_sensor_);
  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  LOG_SENSOR("  ", "Compass", this->compass_sensor_);
}

void BNO055Component::update() {
  if (this->error_code_ != NONE) {
    return;
  }
  
  if (!this->read_sensor_data_()) {
    ESP_LOGW(TAG, "Failed to read sensor data");
    return;
  }
}

bool BNO055Component::read_sensor_data_() {
  uint8_t data[45];
  
  if (!this->read_bytes_(BNO055_ACCEL_DATA_X_LSB_ADDR, data, 45)) {
    return false;
  }
  
  // Convert raw data to sensor values
  int16_t accel_x = (int16_t)((data[1] << 8) | data[0]);
  int16_t accel_y = (int16_t)((data[3] << 8) | data[2]);
  int16_t accel_z = (int16_t)((data[5] << 8) | data[4]);
  
  int16_t mag_x = (int16_t)((data[7] << 8) | data[6]);
  int16_t mag_y = (int16_t)((data[9] << 8) | data[8]);
  int16_t mag_z = (int16_t)((data[11] << 8) | data[10]);
  
  int16_t gyro_x = (int16_t)((data[13] << 8) | data[12]);
  int16_t gyro_y = (int16_t)((data[15] << 8) | data[14]);
  int16_t gyro_z = (int16_t)((data[17] << 8) | data[16]);
  
  int16_t euler_h = (int16_t)((data[19] << 8) | data[18]);
  int16_t euler_r = (int16_t)((data[21] << 8) | data[20]);
  int16_t euler_p = (int16_t)((data[23] << 8) | data[22]);
  
  int16_t linear_accel_x = (int16_t)((data[29] << 8) | data[28]);
  int16_t linear_accel_y = (int16_t)((data[31] << 8) | data[30]);
  int16_t linear_accel_z = (int16_t)((data[33] << 8) | data[32]);
  
  int16_t gravity_x = (int16_t)((data[35] << 8) | data[34]);
  int16_t gravity_y = (int16_t)((data[37] << 8) | data[36]);
  int16_t gravity_z = (int16_t)((data[39] << 8) | data[38]);
  
  int8_t temperature = (int8_t)data[40];
  
  // Publish sensor values
  if (this->accel_x_sensor_ != nullptr)
    this->accel_x_sensor_->publish_state(accel_x / 100.0f);
  if (this->accel_y_sensor_ != nullptr)
    this->accel_y_sensor_->publish_state(accel_y / 100.0f);
  if (this->accel_z_sensor_ != nullptr)
    this->accel_z_sensor_->publish_state(accel_z / 100.0f);
    
  if (this->gyro_x_sensor_ != nullptr)
    this->gyro_x_sensor_->publish_state(gyro_x / 16.0f);
  if (this->gyro_y_sensor_ != nullptr)
    this->gyro_y_sensor_->publish_state(gyro_y / 16.0f);
  if (this->gyro_z_sensor_ != nullptr)
    this->gyro_z_sensor_->publish_state(gyro_z / 16.0f);
    
  if (this->mag_x_sensor_ != nullptr)
    this->mag_x_sensor_->publish_state(mag_x / 16.0f);
  if (this->mag_y_sensor_ != nullptr)
    this->mag_y_sensor_->publish_state(mag_y / 16.0f);
  if (this->mag_z_sensor_ != nullptr)
    this->mag_z_sensor_->publish_state(mag_z / 16.0f);
    
  if (this->euler_h_sensor_ != nullptr)
    this->euler_h_sensor_->publish_state(euler_h / 16.0f);
  if (this->euler_r_sensor_ != nullptr)
    this->euler_r_sensor_->publish_state(euler_r / 16.0f);
  if (this->euler_p_sensor_ != nullptr)
    this->euler_p_sensor_->publish_state(euler_p / 16.0f);
    
  if (this->linear_accel_x_sensor_ != nullptr)
    this->linear_accel_x_sensor_->publish_state(linear_accel_x / 100.0f);
  if (this->linear_accel_y_sensor_ != nullptr)
    this->linear_accel_y_sensor_->publish_state(linear_accel_y / 100.0f);
  if (this->linear_accel_z_sensor_ != nullptr)
    this->linear_accel_z_sensor_->publish_state(linear_accel_z / 100.0f);
    
  if (this->gravity_x_sensor_ != nullptr)
    this->gravity_x_sensor_->publish_state(gravity_x / 100.0f);
  if (this->gravity_y_sensor_ != nullptr)
    this->gravity_y_sensor_->publish_state(gravity_y / 100.0f);
  if (this->gravity_z_sensor_ != nullptr)
    this->gravity_z_sensor_->publish_state(gravity_z / 100.0f);
    
  if (this->temperature_sensor_ != nullptr)
    this->temperature_sensor_->publish_state(temperature);
    
  // Calculate compass heading (magnetic north) with tilt compensation
  if (this->compass_sensor_ != nullptr) {
    float compass_heading = euler_h / 16.0f;
    // Normalize to 0-360 degrees
    if (compass_heading < 0) {
      compass_heading += 360.0f;
    }
    // Tilt compensation is handled automatically by BNO055 in NDOF mode
    this->compass_sensor_->publish_state(compass_heading);
  }
    
  return true;
}

bool BNO055Component::write_byte_(uint8_t register_address, uint8_t value) {
  return this->write(&register_address, 1, &value, 1);
}

bool BNO055Component::read_byte_(uint8_t register_address, uint8_t *value) {
  return this->read(&register_address, 1, value, 1);
}

bool BNO055Component::read_bytes_(uint8_t register_address, uint8_t *data, size_t len) {
  return this->read(&register_address, 1, data, len);
}

}  // namespace bno055
}  // namespace esphome 