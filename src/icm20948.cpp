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

#include "Icm20948.h"  // NOLINT
#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#else
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include "core/core.h"
#endif

namespace bfs {

void Icm20948::Config(TwoWire *i2c, const I2cAddr addr) {
  imu_.Config(i2c, static_cast<uint8_t>(addr));
  iface_ = I2C;
}
void Icm20948::Config(SPIClass *spi, const uint8_t cs) {
  imu_.Config(spi, cs);
  iface_ = SPI;
}
bool Icm20948::Begin() {
  return Begin(MAG_PASSTHROUGH);
}
bool Icm20948::Begin(const MagMode mode) {
  mag_mode_ = mode;
  imu_.Begin();
  if (iface_ == SPI) {
    /* I2C IF DIS */
    WriteRegister(BANK_0_, USER_CTRL_, USER_CTRL_I2C_IF_DIS_);
  }
  /* Select clock source */
  if (!WriteRegister(BANK_0_, PWR_MGMT_1_, PWR_MGMT_1_CLKSEL_AUTO_)) {
    return false;
  }
  /* Reset IMU */
  WriteRegister(BANK_0_, PWR_MGMT_1_, PWR_MGMT_1_RESET_);
  /* Wait for IMU to come back up */
  delay(100);
  if (iface_ == SPI) {
    /* I2C IF DIS */
    WriteRegister(BANK_0_, USER_CTRL_, USER_CTRL_I2C_IF_DIS_);
  }
  /* Select clock source */
  if (!WriteRegister(BANK_0_, PWR_MGMT_1_, PWR_MGMT_1_CLKSEL_AUTO_)) {
    return false;
  }
  /* Check the WHO AM I byte */
  if (!ReadRegisters(BANK_0_, WHO_AM_I_, sizeof(who_am_i_), &who_am_i_)) {
    return false;
  }
  if ((who_am_i_ != WHOAMI_ICM20649_)) {
    return false;
  }
  /* align odr enable */
  if (!WriteRegister(BANK_2_, ODR_ALIGN_EN_, ODR_ALIGN_EN_ALIGN_ENABLE_)) {
    return false;
  }
  /* Set the accel range to default */
  if (!ConfigAccelRange(ACCEL_RANGE_16G)) {
    return false;
  }
  /* Set the gyro range to default*/
  if (!ConfigGyroRange(GYRO_RANGE_2000DPS)) {
    return false;
  }
  /* Set the AccelDLPF to default */
  if (!ConfigAccelDlpfBandwidth(ACCEL_DLPF_BANDWIDTH_473HZ)) {
    return false;
  }
  /* Set the Gyro DLPF to default */
  if (!ConfigGyroDlpfBandwidth(GYRO_DLPF_BANDWIDTH_361HZ)) {
    return false;
  }
  /* Set the Temp DLPF */
  if (!ConfigTempDlpfBandwidth(TEMP_DLPF_BANDWIDTH_217HZ)) {
    return false;
  }
  /* Set the SRD to 0 by default */
  if (!ConfigSrd(0)) {
    return false;
  }
  if (mag_mode_ == MAG_PASSTHROUGH) {
    if (!WriteRegister(BANK_0_, INT_PIN_CFG_, INT_PIN_CFG_BYPASS_EN_)) {
      return false;
    } 
  }
  return true;
}
bool Icm20948::EnableDrdyInt() {
  if (!WriteRegister(BANK_0_, INT_ENABLE_1_, INT_ENABLE_1_INT_ENABLE_)) {
    return false;
  }
  return true;
}
bool Icm20948::DisableDrdyInt() {
  if (!WriteRegister(BANK_0_, INT_ENABLE_1_, INT_ENABLE_1_INT_DISABLE_)) {
    return false;
  }
  return true;
}
bool Icm20948::EnableFifo() {
  if (!WriteRegister(BANK_0_, USER_CTRL_, USER_CTRL_FIFO_EN_)) {
    return false;
  }
  if (!WriteRegister(BANK_0_, FIFO_EN_2_, FIFO_EN_2_GYRO_EN_ | FIFO_EN_2_ACCEL_EN_)) {
    return false;
  }
  return true;
}
bool Icm20948::DisableFifo() {
  if (!WriteRegister(BANK_0_, USER_CTRL_, USER_CTRL_FIFO_DISABLE_)) {
    return false;
  }
  if (!WriteRegister(BANK_0_, FIFO_EN_2_, FIFO_EN_2_DISABLE_ALL_)) {
    return false;
  }
  return true;
}
bool Icm20948::ConfigAccelRange(const AccelRange range) {
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
  ReadRegisters(BANK_2_, ACCEL_CONFIG_, 1, data_buf_);
  data_buf_[0] &= ~(0x06);
  data_buf_[0] |= (requested_accel_range_ << 1);
  if (!WriteRegister(BANK_2_, ACCEL_CONFIG_, data_buf_[0] )) {
    return false;
  }
  /* Update stored range and scale */
  accel_range_ = requested_accel_range_;
  accel_scale_ = requested_accel_scale_;
  return true;
}
bool Icm20948::ConfigGyroRange(const GyroRange range) {
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
  ReadRegisters(BANK_2_, GYRO_CONFIG_1_, 1, data_buf_);
  data_buf_[0] &= ~(0x06);
  data_buf_[0] |= (requested_accel_range_ << 1);
  if (!WriteRegister(BANK_2_, GYRO_CONFIG_1_, data_buf_[0])) {
    return false;
  }
  /* Update stored range and scale */
  gyro_range_ = requested_gyro_range_;
  gyro_scale_ = requested_gyro_scale_;
  return true;
}
bool Icm20948::ConfigSrd(const uint8_t srd) {
  if (!WriteRegister(BANK_2_, ACCEL_SMPLRT_DIV_2_, srd)) {
    return false;
  }
  if (!WriteRegister(BANK_2_, GYRO_SMPLRT_DIV_, srd)) {
    return false;
  }
  srd_ = srd;
  return true;
}
bool Icm20948::ConfigAccelDlpfBandwidth(const AccelDlpfBandwidth dlpf) {
  /* Check input is valid and set requested dlpf */
  switch (dlpf) {
    case ACCEL_DLPF_BANDWIDTH_246HZ: {
      accel_requested_dlpf_ = dlpf;
      break;
    }
    case ACCEL_DLPF_BANDWIDTH_111HZ: {
      accel_requested_dlpf_ = dlpf;
      break;
    }
    case ACCEL_DLPF_BANDWIDTH_50HZ: {
      accel_requested_dlpf_ = dlpf;
      break;
    }
    case ACCEL_DLPF_BANDWIDTH_23HZ: {
      accel_requested_dlpf_ = dlpf;
      break;
    }
    case ACCEL_DLPF_BANDWIDTH_11HZ: {
      accel_requested_dlpf_ = dlpf;
      break;
    }
    case ACCEL_DLPF_BANDWIDTH_5HZ: {
      accel_requested_dlpf_ = dlpf;
      break;
    }
    case ACCEL_DLPF_BANDWIDTH_473HZ: {
      accel_requested_dlpf_ = dlpf;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try setting the dlpf */
  ReadRegisters(BANK_2_, ACCEL_CONFIG_, 1, data_buf_);
  data_buf_[0] |= 0x01;
  data_buf_[0] &= 0xC7;
  data_buf_[0] |= (accel_requested_dlpf_ << 3);
  if (!WriteRegister(BANK_2_, ACCEL_CONFIG_, data_buf_[0])) {
    return false;
  }
  /* Update stored dlpf */
  accel_dlpf_bandwidth_ = accel_requested_dlpf_;
  return true;
}
bool Icm20948::ConfigGyroDlpfBandwidth(const GyroDlpfBandwidth dlpf) {
  /* Check input is valid and set requested dlpf */
  switch (dlpf) {
    case GYRO_DLPF_BANDWIDTH_196HZ: {
      gyro_requested_dlpf_ = dlpf;
      break;
    }
    case GYRO_DLPF_BANDWIDTH_151HZ: {
      gyro_requested_dlpf_ = dlpf;
      break;
    }
    case GYRO_DLPF_BANDWIDTH_119HZ: {
      gyro_requested_dlpf_ = dlpf;
      break;
    }
    case GYRO_DLPF_BANDWIDTH_51HZ: {
      gyro_requested_dlpf_ = dlpf;
      break;
    }
    case GYRO_DLPF_BANDWIDTH_23HZ: {
      gyro_requested_dlpf_ = dlpf;
      break;
    }
    case GYRO_DLPF_BANDWIDTH_11HZ: {
      gyro_requested_dlpf_ = dlpf;
      break;
    }
    case GYRO_DLPF_BANDWIDTH_5HZ: {
      gyro_requested_dlpf_ = dlpf;
      break;
    }
    case GYRO_DLPF_BANDWIDTH_361HZ: {
      gyro_requested_dlpf_ = dlpf;
      break;
    }
    default: {
      return false;
    }
  }
  /* second change dlpf */
  ReadRegisters(BANK_2_, GYRO_CONFIG_1_, sizeof(data_buf_), data_buf_);
  data_buf_[0] |= 0x01;
  data_buf_[0] &= 0xC7;
  data_buf_[0] |= (gyro_requested_dlpf_<< 3);
  if (!WriteRegister(BANK_2_, GYRO_CONFIG_1_, data_buf_[0])) {
    return false;
  }
  /* Update stored dlpf */
  gyro_dlpf_bandwidth_ = gyro_requested_dlpf_;
  return true;
}
bool Icm20948::ConfigTempDlpfBandwidth(const TempDlpfBandwidth dlpf) {
  /* Check input is valid and set requested dlpf */
  switch (dlpf) {
    case TEMP_DLPF_BANDWIDTH_217HZ: {
      temp_requested_dlpf_ = dlpf;
      break;
    }
    case TEMP_DLPF_BANDWIDTH_123HZ: {
      temp_requested_dlpf_ = dlpf;
      break;
    }
    case TEMP_DLPF_BANDWIDTH_65HZ: {
      temp_requested_dlpf_ = dlpf;
      break;
    }
    case TEMP_DLPF_BANDWIDTH_34HZ: {
      temp_requested_dlpf_ = dlpf;
      break;
    }
    case TEMP_DLPF_BANDWIDTH_17HZ: {
      temp_requested_dlpf_ = dlpf;
      break;
    }
    default: {
      return false;
    }
  }
  /* second change dlpf */
  if (!WriteRegister(BANK_2_, TEMP_CONFIG_, temp_requested_dlpf_)) {
    return false;
  }
  /* Update stored dlpf */
  temp_dlpf_bandwidth_ = temp_requested_dlpf_;
  return true;
}
bool Icm20948::Read() {
  /* Reset the new data flags */
  new_imu_data_ = false;
  /* Read the data registers */
  if (!ReadRegisters(BANK_0_, INT_STATUS_1_, 1, data_buf_)) {
    return false;
  }
  /* Check if data is ready */
  new_imu_data_ = (data_buf_[0] & RAW_DATA_RDY_INT_);
  if (!new_imu_data_) {
    return false;
  }
  ReadRegisters(BANK_0_, ACCEL_XOUT_H, sizeof(data_buf_), data_buf_);
  /* Unpack the buffer */
  accel_cnts_[0] = static_cast<int16_t>(data_buf_[0])  << 8 | data_buf_[1];
  accel_cnts_[1] = static_cast<int16_t>(data_buf_[2])  << 8 | data_buf_[3];
  accel_cnts_[2] = static_cast<int16_t>(data_buf_[4])  << 8 | data_buf_[5];
  gyro_cnts_[0] =  static_cast<int16_t>(data_buf_[6])  << 8 | data_buf_[7];
  gyro_cnts_[1] =  static_cast<int16_t>(data_buf_[8])  << 8 | data_buf_[9];
  gyro_cnts_[2] =  static_cast<int16_t>(data_buf_[10]) << 8 | data_buf_[11];
  temp_cnts_ =     static_cast<int16_t>(data_buf_[12])  << 8 | data_buf_[13];
  /* Convert to float values and rotate the accel / gyro axis */
  accel_[0] = static_cast<float>(accel_cnts_[1]) * accel_scale_ * G_MPS2_;
  accel_[1] = static_cast<float>(accel_cnts_[0]) * accel_scale_ * G_MPS2_;
  accel_[2] = static_cast<float>(accel_cnts_[2]) * accel_scale_ * -1.0f * G_MPS2_;
  temp_ = (static_cast<float>(temp_cnts_) - 21.0f) / TEMP_SCALE_ + 21.0f;
  gyro_[0] = static_cast<float>(gyro_cnts_[1]) * gyro_scale_ * DEG2RAD_;
  gyro_[1] = static_cast<float>(gyro_cnts_[0]) * gyro_scale_ * DEG2RAD_;
  gyro_[2] = static_cast<float>(gyro_cnts_[2]) * gyro_scale_ * -1.0f * DEG2RAD_;
  return true;
}
int16_t Icm20948::ReadFifo(uint8_t * const data, const size_t len) {
  if (!data) {
    return -1;
  }
  /* Read the FIFO interrupt */
  if (!ReadRegisters(BANK_0_, INT_STATUS_2_, 1, data_buf_)) {
    return -1;
  }
  /* Check for fifo overflow */
  fifo_overflowed_ = (data_buf_[0] & FIFO_OFLOW_INT_);
  /* FIFO Count */
  if (!ReadRegisters(BANK_0_, FIFO_COUNT_H_, 2, data_buf_)) {
    return -1;
  }
  fifo_count_ = static_cast<int16_t>(data_buf_[0]) << 8 | data_buf_[1];
  if (fifo_count_ > 0) {
    if (len < fifo_count_) {
      bytes_to_read_ = len;
    } else {
      bytes_to_read_ = fifo_count_;
    }
    if (!ReadFifo(BANK_0_, FIFO_R_W_, bytes_to_read_, data)) {
      return -1;
    }
    return bytes_to_read_;
  } else {
    return 0;
  }
}
int16_t Icm20948::ProcessFifoData(uint8_t * const data, const size_t len,
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
bool Icm20948::WriteRegister(const uint8_t bank, const uint8_t reg,
                             const uint8_t data) {
  if(bank != current_bank_) {
    if (!imu_.WriteRegister(REG_BANK_SEL_, bank << 4, SPI_CLOCK_)) {
      return false;
    } else {
      current_bank_ = bank;
    }
  }
  return imu_.WriteRegister(reg, data, SPI_CLOCK_);
}
bool Icm20948::ReadRegisters(const uint8_t bank, const uint8_t reg, const uint8_t count,
                             uint8_t * const data) {
  if(bank != current_bank_) {
    if (!imu_.WriteRegister(REG_BANK_SEL_, bank << 4, SPI_CLOCK_)) {
      return false;
    } else {
      current_bank_ = bank;
    }
  }
  return imu_.ReadRegisters(reg, count, SPI_CLOCK_, data);
}
bool Icm20948::ReadFifo(const uint8_t bank, const uint8_t reg, const uint8_t count,
                        uint8_t * const data) {
  if(bank != current_bank_) {
    if (!imu_.WriteRegister(REG_BANK_SEL_, bank << 4, SPI_CLOCK_)) {
      return false;
    } else {
      current_bank_ = bank;
    }
  }
  return imu_.ReadFifo(reg, count, SPI_CLOCK_, data);
}

}  // namespace bfs
