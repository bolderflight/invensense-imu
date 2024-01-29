/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2024 Bolder Flight Systems Inc
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

#include "mpu6050.h"  // NOLINT
#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#else
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include "core/core.h"
#endif

namespace bfs {

void Mpu6050::Config(TwoWire *i2c, const I2cAddr addr) {
  imu_.Config(i2c, static_cast<uint8_t>(addr));
}
bool Mpu6050::Begin() {
  imu_.Begin();
  /* Reset the IMU */
  WriteRegister(PWR_MGMNT_1_, H_RESET_);
  /* Wait for IMU to come back up */
  delay(100);
  /* Select clock source to gyro */
  if (!WriteRegister(PWR_MGMNT_1_, CLKSEL_PLL_)) {
    return false;
  }
  /* Check the WHO AM I byte */
  if (!ReadRegisters(WHOAMI_, sizeof(who_am_i_), &who_am_i_)) {
    return false;
  }
  if (who_am_i_ != WHOAMI_MPU6050_) {
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
  /* Set the DLPF to 184HZ by default */
  if (!ConfigDlpfBandwidth(DLPF_BANDWIDTH_184HZ)) {
    return false;
  }
  /* Set the SRD to 0 by default */
  if (!ConfigSrd(0)) {
    return false;
  }
  return true;
}
bool Mpu6050::EnableDrdyInt() {
  if (!WriteRegister(INT_PIN_CFG_, INT_PULSE_50US_)) {
    return false;
  }
  if (!WriteRegister(INT_ENABLE_, INT_RAW_RDY_EN_)) {
    return false;
  }
  return true;
}
bool Mpu6050::DisableDrdyInt() {
  if (!WriteRegister(INT_ENABLE_, INT_DISABLE_)) {
    return false;
  }
  return true;
}
bool Mpu6050::EnableFifo() {
  if (!WriteRegister(USER_CTRL_, USER_CTRL_FIFO_EN_)) {
    return false;
  }
  if (!WriteRegister(FIFO_EN_, FIFO_EN_GYRO_ | FIFO_EN_ACCEL_)) {
    return false;
  }
  return true;
}
bool Mpu6050::DisableFifo() {
  if (!WriteRegister(USER_CTRL_, USER_CTRL_FIFO_DISABLE_)) {
    return false;
  }
  if (!WriteRegister(FIFO_EN_, FIF_EN_DISABLE_ALL_)) {
    return false;
  }
  return true;
}
void Mpu6050::ResetFifo() {
  WriteRegister(USER_CTRL_, USER_CTRL_FIFO_RESET_);
}
bool Mpu6050::ConfigAccelRange(const AccelRange range) {
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
bool Mpu6050::ConfigGyroRange(const GyroRange range) {
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
bool Mpu6050::ConfigSrd(const uint8_t srd) {
  /* Set the IMU sample rate */
  if (!WriteRegister(SMPLRT_DIV_, srd)) {
    return false;
  }
  srd_ = srd;
  return true;
}
bool Mpu6050::ConfigDlpfBandwidth(const DlpfBandwidth dlpf) {
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
bool Mpu6050::Read() {
  /* Reset the new data flags */
  new_imu_data_ = false;
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
  /* Convert to float values and rotate the accel / gyro axis */
  accel_[0] = static_cast<float>(accel_cnts_[1]) * accel_scale_ * G_MPS2_;
  accel_[1] = static_cast<float>(accel_cnts_[0]) * accel_scale_ * G_MPS2_;
  accel_[2] = static_cast<float>(accel_cnts_[2]) * accel_scale_ * -1.0f *
              G_MPS2_;
  temp_ = (static_cast<float>(temp_cnts_)) / TEMP_SCALE_ + 36.53f;
  gyro_[0] = static_cast<float>(gyro_cnts_[1]) * gyro_scale_ * DEG2RAD_;
  gyro_[1] = static_cast<float>(gyro_cnts_[0]) * gyro_scale_ * DEG2RAD_;
  gyro_[2] = static_cast<float>(gyro_cnts_[2]) * gyro_scale_ * -1.0f * DEG2RAD_;
  return true;
}
int16_t Mpu6050::ReadFifo(uint8_t * const data, const size_t len) {
  if (!data) {
    return -1;
  }
  /* Read the FIFO interrupt */
  if (!ReadRegisters(INT_STATUS_, 1, data_buf_)) {
    return -1;
  }
  /* Check for fifo overflow */
  fifo_overflowed_ = (data_buf_[0] & FIFO_OFLOW_INT_);
  /* FIFO Count */
  if (!ReadRegisters(FIFO_COUNT_H_, 2, data_buf_)) {
    return -1;
  }
  fifo_count_ = static_cast<int16_t>(data_buf_[0]) << 8 | data_buf_[1];
  if (fifo_count_ > 0) {
    if (len < fifo_count_) {
      bytes_to_read_ = len;
    } else {
      bytes_to_read_ = fifo_count_;
    }
    if (!ReadFifo(FIFO_R_W_, bytes_to_read_, data)) {
      return -1;
    }
    return bytes_to_read_;
  } else {
    return 0;
  }
}
int16_t Mpu6050::ProcessFifoData(uint8_t * const data, const size_t len,
                                 float * const gx, float * const gy, float * const gz,
                                 float * const ax, float * const ay, float * const az) {
  if ((!data) || (!gx) || (!gy) || (!gz) || (!ax) || (!ay) || (!az)) {
    return -1;
  }
  size_t j = 0;
  for (size_t i = 0; i < len; i = i + 12) {
    /* Unpack the buffer */
    accel_cnts_[0] = static_cast<int16_t>(data[i + 0])  << 8 | data[i + 1];
    accel_cnts_[1] = static_cast<int16_t>(data[i + 2])  << 8 | data[i + 3];
    accel_cnts_[2] = static_cast<int16_t>(data[i + 4])  << 8 | data[i + 5];
    gyro_cnts_[0] =  static_cast<int16_t>(data[i + 6])  << 8 | data[i + 7];
    gyro_cnts_[1] =  static_cast<int16_t>(data[i + 8]) << 8 | data[i + 9];
    gyro_cnts_[2] =  static_cast<int16_t>(data[i + 10]) << 8 | data[i + 11];
    /* Convert to float values and rotate the accel / gyro axis */
    ax[j] = static_cast<float>(accel_cnts_[1]) * accel_scale_ * G_MPS2_;
    ay[j] = static_cast<float>(accel_cnts_[0]) * accel_scale_ * G_MPS2_;
    az[j] = static_cast<float>(accel_cnts_[2]) * accel_scale_ * -1.0f * G_MPS2_;
    gx[j] = static_cast<float>(gyro_cnts_[1]) * gyro_scale_ * DEG2RAD_;
    gy[j] = static_cast<float>(gyro_cnts_[0]) * gyro_scale_ * DEG2RAD_;
    gz[j] = static_cast<float>(gyro_cnts_[2]) * gyro_scale_ * -1.0f * DEG2RAD_;
    j++;
  }
  return j;
}
bool Mpu6050::WriteRegister(const uint8_t reg, const uint8_t data) {
  return imu_.WriteRegister(reg, data, 0);
}
bool Mpu6050::ReadRegisters(const uint8_t reg, const uint8_t count,
                            uint8_t * const data) {
  return imu_.ReadRegisters(reg, count, 0, data);
}
bool Mpu6050::ReadFifo(const uint8_t reg, const uint8_t count,
                       uint8_t * const data) {
  return imu_.ReadFifo(reg, count, 0, data);
}

}  // namespace bfs
