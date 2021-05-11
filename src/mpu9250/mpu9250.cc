/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "mpu9250/mpu9250.h"
#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"
#include "units/units.h"

namespace bfs {

bool Mpu9250::Init(const ImuConfig &ref) {
  /* Copy the config */
  config_ = ref;
  /* Determine the interface type */
  if (std::holds_alternative<TwoWire *>(config_.bus)) {
    iface_ = I2C;
    i2c_ = std::get<TwoWire *>(config_.bus);
  } else if (std::holds_alternative<SPIClass *>(config_.bus)) {
    iface_ = SPI;
    spi_ = std::get<SPIClass *>(config_.bus);
  } else {
    return false;
  }
  /* Setup the IMU */
  if (!Begin()) {return false;}
  if (!ConfigAccelRange(Mpu9250::ACCEL_RANGE_16G)) {return false;}
  if (!ConfigGyroRange(Mpu9250::GYRO_RANGE_2000DPS)) {return false;}
  switch (config_.frame_rate) {
    case FRAME_RATE_200HZ: {
      if (!ConfigDlpf(Mpu9250::DLPF_BANDWIDTH_92HZ)) {return false;}
      if (!ConfigSrd(4)) {return false;}
      /* IMU is healhty at 5x the sampling rate or 25 ms */
      imu_health_period_ms_ = 25;
      /* Mag is sampled at 100 Hz, so healthy at 50 ms */
      mag_health_period_ms_ = 50;
      break;
    }
    case FRAME_RATE_100HZ: {
      if (!ConfigDlpf(Mpu9250::DLPF_BANDWIDTH_41HZ)) {return false;}
      if (!ConfigSrd(9)) {return false;}
      /* IMU is healhty at 5x the sampling rate or 50 ms */
      imu_health_period_ms_ = 50;
      /* Mag is sampled at 100 Hz, so healthy at 50 ms */
      mag_health_period_ms_ = 50;
      break;
    }
    case FRAME_RATE_50HZ: {
      if (!ConfigDlpf(Mpu9250::DLPF_BANDWIDTH_20HZ)) {return false;}
      if (!ConfigSrd(19)) {return false;}
      /* IMU is healhty at 5x the sampling rate or 100 ms */
      imu_health_period_ms_ = 100;
      /* Mag is sampled at 8 Hz, so healthy at 625 ms */
      mag_health_period_ms_ = 625;
      break;
    }
  }
  if (!EnableDrdyInt()) {return false;}
  /* Estimate gyro bias */
  time_ms_ = 0;
  while (time_ms_ < INIT_TIME_MS_) {
    if (ReadImu()) {
      gx_.Update(gyro_radps_(0));
      gy_.Update(gyro_radps_(1));
      gz_.Update(gyro_radps_(2));
    }
  }
  /* Assign gyro bias */
  gyro_bias_radps_(0) = -gx_.mean();
  gyro_bias_radps_(1) = -gy_.mean();
  gyro_bias_radps_(2) = -gz_.mean();
  /* Accel bias and scale factor */
  accel_bias_mps2_(0) = config_.accel_bias_mps2[0];
  accel_bias_mps2_(1) = config_.accel_bias_mps2[1];
  accel_bias_mps2_(2) = config_.accel_bias_mps2[2];
  accel_scale_factor_(0, 0) = config_.accel_scale[0][0];
  accel_scale_factor_(0, 1) = config_.accel_scale[0][1];
  accel_scale_factor_(0, 2) = config_.accel_scale[0][2];
  accel_scale_factor_(1, 0) = config_.accel_scale[1][0];
  accel_scale_factor_(1, 1) = config_.accel_scale[1][1];
  accel_scale_factor_(1, 2) = config_.accel_scale[1][2];
  accel_scale_factor_(2, 0) = config_.accel_scale[2][0];
  accel_scale_factor_(2, 1) = config_.accel_scale[2][1];
  accel_scale_factor_(2, 2) = config_.accel_scale[2][2];
  /* Mag bias and scale factor */
  mag_bias_ut_(0) = config_.mag_bias_ut[0];
  mag_bias_ut_(1) = config_.mag_bias_ut[1];
  mag_bias_ut_(2) = config_.mag_bias_ut[2];
  mag_scale_factor_(0, 0) = config_.mag_scale[0][0];
  mag_scale_factor_(0, 1) = config_.mag_scale[0][1];
  mag_scale_factor_(0, 2) = config_.mag_scale[0][2];
  mag_scale_factor_(1, 0) = config_.mag_scale[1][0];
  mag_scale_factor_(1, 1) = config_.mag_scale[1][1];
  mag_scale_factor_(1, 2) = config_.mag_scale[1][2];
  mag_scale_factor_(2, 0) = config_.mag_scale[2][0];
  mag_scale_factor_(2, 1) = config_.mag_scale[2][1];
  mag_scale_factor_(2, 2) = config_.mag_scale[2][2];
  /* Rotation */
  rotation_(0, 0) = config_.rotation[0][0];
  rotation_(0, 1) = config_.rotation[0][1];
  rotation_(0, 2) = config_.rotation[0][2];
  rotation_(1, 0) = config_.rotation[1][0];
  rotation_(1, 1) = config_.rotation[1][1];
  rotation_(1, 2) = config_.rotation[1][2];
  rotation_(2, 0) = config_.rotation[2][0];
  rotation_(2, 1) = config_.rotation[2][1];
  rotation_(2, 2) = config_.rotation[2][2];
  /* Reset timers */
  imu_health_timer_ms_ = 0;
  mag_health_timer_ms_ = 0;
  return true;
}
bool Mpu9250::Read(ImuData * const ptr) {
  if (!ptr) {return false;}
  ptr->new_imu_data = ReadImu();
  ptr->new_mag_data = new_mag_data_;
  ptr->imu_healthy = (imu_health_timer_ms_ < imu_health_period_ms_);
  ptr->mag_healthy = (mag_health_timer_ms_ < mag_health_period_ms_);
  if (ptr->new_imu_data) {
    imu_health_timer_ms_ = 0;
    ptr->accel_mps2[0] = accel_mps2_(0);
    ptr->accel_mps2[1] = accel_mps2_(1);
    ptr->accel_mps2[2] = accel_mps2_(2);
    ptr->gyro_radps[0] = gyro_radps_(0);
    ptr->gyro_radps[1] = gyro_radps_(1);
    ptr->gyro_radps[2] = gyro_radps_(2);
    ptr->die_temp_c = die_temperature_c_;
  }
  if (ptr->new_mag_data) {
    mag_health_timer_ms_ = 0;
    ptr->mag_ut[0] = mag_ut_(0);
    ptr->mag_ut[1] = mag_ut_(1);
    ptr->mag_ut[2] = mag_ut_(2);
  }
  return ptr->new_imu_data;
}
bool Mpu9250::Begin() {
  if (iface_ == SPI) {
    pinMode(config_.dev, OUTPUT);
    /* Toggle CS pin to lock in SPI mode */
    digitalWriteFast(config_.dev, LOW);
    delay(1);
    digitalWriteFast(config_.dev, HIGH);
  }
  spi_clock_ = 1000000;
  /* Select clock source to gyro */
  if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) {
    return false;
  }
  /* Enable I2C master mode */
  if (!WriteRegister(USER_CTRL_, I2C_MST_EN_)) {
    return false;
  }
  /* Set the I2C bus speed to 400 kHz */
  if (!WriteRegister(I2C_MST_CTRL_, I2C_MST_CLK_)) {
    return false;
  }
  /* Set AK8963 to power down */
  WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
  /* Reset the MPU9250 */
  WriteRegister(PWR_MGMNT_1_, H_RESET_);
  /* Wait for MPU-9250 to come back up */
  delay(1);
  /* Reset the AK8963 */
  WriteAk8963Register(AK8963_CNTL2_, AK8963_RESET_);
  /* Select clock source to gyro */
  if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) {
    return false;
  }
  /* Check the WHO AM I byte */
  who_am_i_;
  if (!ReadRegisters(WHOAMI_, sizeof(who_am_i_), &who_am_i_)) {
    return false;
  }
  if ((who_am_i_ != WHOAMI_MPU9250_) && (who_am_i_ != WHOAMI_MPU9255_)) {
    return false;
  }
  /* Enable I2C master mode */
  if (!WriteRegister(USER_CTRL_, I2C_MST_EN_)) {
    return false;
  }
  /* Set the I2C bus speed to 400 kHz */
  if (!WriteRegister(I2C_MST_CTRL_, I2C_MST_CLK_)) {
    return false;
  }
  /* Check the AK8963 WHOAMI */
  if (!ReadAk8963Registers(AK8963_WHOAMI_, sizeof(who_am_i_), &who_am_i_)) {
    return false;
  }
  if (who_am_i_ != WHOAMI_AK8963_) {
    return false;
  }
  /* Get the magnetometer calibration */
  /* Set AK8963 to power down */
  if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_)) {
    return false;
  }
  delay(100);  // long wait between AK8963 mode changes
  /* Set AK8963 to FUSE ROM access */
  if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_FUSE_ROM_)) {
    return false;
  }
  delay(100);  // long wait between AK8963 mode changes
  /* Read the AK8963 ASA registers and compute magnetometer scale factors */
  asa_buff_[3];
  if (!ReadAk8963Registers(AK8963_ASA_, sizeof(asa_buff_), asa_buff_)) {
    return false;
  }
  mag_scale_[0] = ((static_cast<float>(asa_buff_[0]) - 128.0f)
    / 256.0f + 1.0f) * 4912.0f / 32760.0f;
  mag_scale_[1] = ((static_cast<float>(asa_buff_[1]) - 128.0f)
    / 256.0f + 1.0f) * 4912.0f / 32760.0f;
  mag_scale_[2] = ((static_cast<float>(asa_buff_[2]) - 128.0f)
    / 256.0f + 1.0f) * 4912.0f / 32760.0f;
  /* Set AK8963 to power down */
  if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_)) {
    return false;
  }
  /* Set AK8963 to 16 bit resolution, 100 Hz update rate */
  if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_CNT_MEAS2_)) {
    return false;
  }
  delay(100);  // long wait between AK8963 mode changes
  /* Select clock source to gyro */
  if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) {
    return false;
  }
  /* Set the accel range to 16G by default */
  if (!ConfigAccelRange(ACCEL_RANGE_16G)) {
    return false;
  }
  /* Set the gyro range to 2000DPS by default*/
  if (!ConfigGyroRange(GYRO_RANGE_2000DPS)) {
    return false;
  }
  /* Set the DLPF to 20HZ by default */
  if (!ConfigDlpf(DLPF_BANDWIDTH_20HZ)) {
    return false;
  }
  /* Set the SRD to 0 by default */
  if (!ConfigSrd(0)) {
    return false;
  }
  return true;
}
bool Mpu9250::EnableDrdyInt() {
  spi_clock_ = 1000000;
  if (!WriteRegister(INT_PIN_CFG_, INT_PULSE_50US_)) {
    return false;
  }
  if (!WriteRegister(INT_ENABLE_, INT_RAW_RDY_EN_)) {
    return false;
  }
  return true;
}
bool Mpu9250::DisableDrdyInt() {
  spi_clock_ = 1000000;
  if (!WriteRegister(INT_ENABLE_, INT_DISABLE_)) {
    return false;
  }
  return true;
}
bool Mpu9250::ConfigAccelRange(const AccelRange range) {
  spi_clock_ = 1000000;
  /* Check input is valid and set requested range and scale */
  switch (range) {
    case ACCEL_RANGE_2G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 2.0f / 32767.5f;
      break;
    }
    case ACCEL_RANGE_4G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 4.0f / 32767.5f;
      break;
    }
    case ACCEL_RANGE_8G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 8.0f / 32767.5f;
      break;
    }
    case ACCEL_RANGE_16G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 16.0f / 32767.5f;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try setting the requested range */
  if (!WriteRegister(ACCEL_CONFIG_, requested_accel_range_)) {
    return false;
  }
  /* Update stored range and scale */
  accel_range_ = requested_accel_range_;
  accel_scale_ = requested_accel_scale_;
  return true;
}
bool Mpu9250::ConfigGyroRange(const GyroRange range) {
  spi_clock_ = 1000000;
  /* Check input is valid and set requested range and scale */
  switch (range) {
    case GYRO_RANGE_250DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 250.0f / 32767.5f;
      break;
    }
    case GYRO_RANGE_500DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 500.0f / 32767.5f;
      break;
    }
    case GYRO_RANGE_1000DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 1000.0f / 32767.5f;
      break;
    }
    case GYRO_RANGE_2000DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 2000.0f / 32767.5f;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try setting the requested range */
  if (!WriteRegister(GYRO_CONFIG_, requested_gyro_range_)) {
    return false;
  }
  /* Update stored range and scale */
  gyro_range_ = requested_gyro_range_;
  gyro_scale_ = requested_gyro_scale_;
  return true;
}
bool Mpu9250::ConfigSrd(const uint8_t srd) {
  spi_clock_ = 1000000;
  /* Changing the SRD to allow us to set the magnetometer successfully */
  if (!WriteRegister(SMPLRT_DIV_, 19)) {
    return false;
  }
  /* Set the magnetometer sample rate */
  if (srd > 9) {
    /* Set AK8963 to power down */
    WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
    delay(100);  // long wait between AK8963 mode changes
    /* Set AK8963 to 16 bit resolution, 8 Hz update rate */
    if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_CNT_MEAS1_)) {
      return false;
    }
    delay(100);  // long wait between AK8963 mode changes
    /* Instruct the MPU9250 to get 8 bytes from the AK8963 at the sample rate */
    if (!ReadAk8963Registers(AK8963_ST1_, sizeof(mag_data_), mag_data_)) {
      return false;
    }
  } else {
    /* Set AK8963 to power down */
    WriteAk8963Register(AK8963_CNTL1_, AK8963_PWR_DOWN_);
    delay(100);  // long wait between AK8963 mode changes
    /* Set AK8963 to 16 bit resolution, 100 Hz update rate */
    if (!WriteAk8963Register(AK8963_CNTL1_, AK8963_CNT_MEAS2_)) {
      return false;
    }
    delay(100);  // long wait between AK8963 mode changes
    /* Instruct the MPU9250 to get 8 bytes from the AK8963 at the sample rate */
    if (!ReadAk8963Registers(AK8963_ST1_, sizeof(mag_data_), mag_data_)) {
      return false;
    }
  }
  /* Set the IMU sample rate */
  if (!WriteRegister(SMPLRT_DIV_, srd)) {
    return false;
  }
  srd_ = srd;
  return true;
}
bool Mpu9250::ConfigDlpf(const DlpfBandwidth dlpf) {
  spi_clock_ = 1000000;
  /* Check input is valid and set requested dlpf */
  switch (dlpf) {
    case DLPF_BANDWIDTH_184HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_92HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_41HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_20HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_10HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    case DLPF_BANDWIDTH_5HZ: {
      requested_dlpf_ = dlpf;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try setting the dlpf */
  if (!WriteRegister(ACCEL_CONFIG2_, requested_dlpf_)) {
    return false;
  }
  if (!WriteRegister(CONFIG_, requested_dlpf_)) {
    return false;
  }
  /* Update stored dlpf */
  dlpf_bandwidth_ = requested_dlpf_;
  return true;
}
bool Mpu9250::ReadImu() {
  spi_clock_ = 20000000;
  /* Reset the new_mag_data flag */
  new_mag_data_ = false;
  /* Read the data registers */
  if (!ReadRegisters(INT_STATUS_, sizeof(data_buf_), data_buf_)) {
    return false;
  }
  /* Check if data is ready */
  new_imu_data_ = (data_buf_[0] & RAW_DATA_RDY_INT_);
  if (!new_imu_data_) {
    return false;
  }
  /* Unpack the buffer */
  accel_cnts_[0] = static_cast<int16_t>(data_buf_[1])  << 8 | data_buf_[2];
  accel_cnts_[1] = static_cast<int16_t>(data_buf_[3])  << 8 | data_buf_[4];
  accel_cnts_[2] = static_cast<int16_t>(data_buf_[5])  << 8 | data_buf_[6];
  temp_cnts_ =     static_cast<int16_t>(data_buf_[7])  << 8 | data_buf_[8];
  gyro_cnts_[0] =  static_cast<int16_t>(data_buf_[9])  << 8 | data_buf_[10];
  gyro_cnts_[1] =  static_cast<int16_t>(data_buf_[11]) << 8 | data_buf_[12];
  gyro_cnts_[2] =  static_cast<int16_t>(data_buf_[13]) << 8 | data_buf_[14];
  new_mag_data_ = (data_buf_[15] & AK8963_DATA_RDY_INT_);
  mag_cnts_[0] =   static_cast<int16_t>(data_buf_[17]) << 8 | data_buf_[16];
  mag_cnts_[1] =   static_cast<int16_t>(data_buf_[19]) << 8 | data_buf_[18];
  mag_cnts_[2] =   static_cast<int16_t>(data_buf_[21]) << 8 | data_buf_[20];
  /* Check for mag overflow */
  mag_sensor_overflow_ = (data_buf_[22] & AK8963_HOFL_);
  if (mag_sensor_overflow_) {
    new_mag_data_ = false;
  }
  /* Convert to float values and rotate the accel / gyro axis */
  accel_(0) = convacc(static_cast<float>(accel_cnts_[1]) * accel_scale_,
                      LinAccUnit::G, LinAccUnit::MPS2);
  accel_(2) = convacc(static_cast<float>(accel_cnts_[2]) * accel_scale_ *
                      -1.0f, LinAccUnit::G, LinAccUnit::MPS2);
  accel_(1) = convacc(static_cast<float>(accel_cnts_[0]) * accel_scale_,
                      LinAccUnit::G, LinAccUnit::MPS2);
  die_temperature_c_ = (static_cast<float>(temp_cnts_) - 21.0f) / temp_scale_
                        + 21.0f;
  gyro_(1) =  deg2rad(static_cast<float>(gyro_cnts_[0]) * gyro_scale_);
  gyro_(0) =  deg2rad(static_cast<float>(gyro_cnts_[1]) * gyro_scale_);
  gyro_(2) =  deg2rad(static_cast<float>(gyro_cnts_[2]) * gyro_scale_ * -1.0f);
  mag_(0) =   static_cast<float>(mag_cnts_[0]) * mag_scale_[0];
  mag_(1) =   static_cast<float>(mag_cnts_[1]) * mag_scale_[1];
  mag_(2) =   static_cast<float>(mag_cnts_[2]) * mag_scale_[2];
  /* Apply rotation */
  accel_mps2_ = accel_scale_factor_ * rotation_ * accel_ + accel_bias_mps2_;
  gyro_radps_ = rotation_ * gyro_ + gyro_bias_radps_;
  /* Only update on new data */
  if (new_mag_data_) {
    mag_ut_ = mag_scale_factor_ * rotation_ * mag_ + mag_bias_ut_;
  }
  return true;
}
bool Mpu9250::WriteRegister(uint8_t reg, uint8_t data) {
  uint8_t ret_val;
  if (iface_ == I2C) {
    i2c_->beginTransmission(config_.dev);
    i2c_->write(reg);
    i2c_->write(data);
    i2c_->endTransmission();
  } else {
    spi_->beginTransaction(SPISettings(spi_clock_, MSBFIRST, SPI_MODE3));
    digitalWriteFast(config_.dev, LOW);
    #if defined(__IMXRT1062__)
      delayNanoseconds(50);
    #endif
    spi_->transfer(reg);
    spi_->transfer(data);
    digitalWriteFast(config_.dev, HIGH);
    #if defined(__IMXRT1062__)
      delayNanoseconds(50);
    #endif
    spi_->endTransaction();
  }
  delay(10);
  ReadRegisters(reg, sizeof(ret_val), &ret_val);
  if (data == ret_val) {
    return true;
  } else {
    return false;
  }
}
bool Mpu9250::ReadRegisters(uint8_t reg, uint8_t count, uint8_t *data) {
  if (iface_ == I2C) {
    i2c_->beginTransmission(config_.dev);
    i2c_->write(reg);
    i2c_->endTransmission(false);
    uint8_t bytes_rx =
      i2c_->requestFrom(static_cast<uint8_t>(config_.dev), count);
    if (bytes_rx == count) {
      for (std::size_t i = 0; i < count; i++) {
        data[i] = i2c_->read();
      }
      return true;
    } else {
      return false;
    }
  } else {
    spi_->beginTransaction(SPISettings(spi_clock_, MSBFIRST, SPI_MODE3));
    digitalWriteFast(config_.dev, LOW);
    #if defined(__IMXRT1062__)
      delayNanoseconds(50);
    #endif
    spi_->transfer(reg | SPI_READ_);
    spi_->transfer(data, count);
    digitalWriteFast(config_.dev, HIGH);
    #if defined(__IMXRT1062__)
      delayNanoseconds(50);
    #endif
    spi_->endTransaction();
    return true;
  }
}
bool Mpu9250::WriteAk8963Register(uint8_t reg, uint8_t data) {
  uint8_t ret_val;
  if (!WriteRegister(I2C_SLV0_ADDR_, AK8963_I2C_ADDR_)) {
    return false;
  }
  if (!WriteRegister(I2C_SLV0_REG_, reg)) {
    return false;
  }
  if (!WriteRegister(I2C_SLV0_DO_, data)) {
    return false;
  }
  if (!WriteRegister(I2C_SLV0_CTRL_, I2C_SLV0_EN_ | sizeof(data))) {
    return false;
  }
  if (!ReadAk8963Registers(reg, sizeof(ret_val), &ret_val)) {
    return false;
  }
  if (data == ret_val) {
    return true;
  } else {
    return false;
  }
}
bool Mpu9250::ReadAk8963Registers(uint8_t reg, uint8_t count, uint8_t *data) {
  if (!WriteRegister(I2C_SLV0_ADDR_, AK8963_I2C_ADDR_ | I2C_READ_FLAG_)) {
    return false;
  }
  if (!WriteRegister(I2C_SLV0_REG_, reg)) {
    return false;
  }
  if (!WriteRegister(I2C_SLV0_CTRL_, I2C_SLV0_EN_ | count)) {
    return false;
  }
  delay(1);
  return ReadRegisters(EXT_SENS_DATA_00_, count, data);
}

}  // namespace bfs
