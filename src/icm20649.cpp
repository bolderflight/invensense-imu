/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
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

#include "Icm20649.h"  // NOLINT
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

void Icm20649::Config(TwoWire *i2c, const I2cAddr addr) {
  imu_.Config(i2c, static_cast<uint8_t>(addr));
}
void Icm20649::Config(SPIClass *spi, const uint8_t cs) {
  imu_.Config(spi, cs);
}
bool Icm20649::Begin() {
  delay(1000);
  imu_.Begin();
  /* 1 MHz for config */
  spi_clock_ = SPI_CFG_CLOCK_;
  /* I2C IF DIS */
  WriteRegister(0x10, USER_CTRL);
  /* Set to Bank 0 */
  if(!setBank(0)) {
    Serial.println("1");
    return false;
  }
  delay(100);
  /* Select clock source to gyro */
  if (!WriteRegister(PWR_MGMT_1, CLKSEL_PLL_)) {
    Serial.println("2");
    return false;
  }
  /* Set to Bank 0 */
  if(!setBank(0)) {
    Serial.println("3");
    return false;
  }
  /* Check the WHO AM I byte */
  if (!ReadRegisters(WHO_AM_I, sizeof(who_am_i_), &who_am_i_)) {
    Serial.println("4");
    return false;
  }
  if ((who_am_i_ != WHOAMI_ICM20649_)) {
    Serial.println("5");
    return false;
  }
  /* align odr enable */
  setBank(2);
  if (!WriteRegister(ODR_ALIGN_EN, 0x01)) {
    Serial.println("6");
    return false;
  }
  if(!setBank(0)) {
    Serial.println("7");
    return false;
  }
  if (!WriteRegister(PWR_MGMT_1, CLKSEL_PLL_)) {
    Serial.println("8");
    return false;
  }
  /* Set the accel range to 16G by default */
  if (!ConfigAccelRange(ACCEL_RANGE_16G)) {
    Serial.println("9");
    return false;
  }
  /* Set the gyro range to 2000DPS by default*/
  if (!ConfigGyroRange(GYRO_RANGE_2000DPS)) {
    Serial.println("10");
    return false;
  }
  /* Set the AccelDLPF to 111HZ by default */
  if (!ConfigAccelDlpfBandwidth(ACCEL_DLPF_BANDWIDTH_111HZ)) {
    Serial.println("11");
    return false;
  }
  /* Set the Gyro DLPF to 184HZ by default */
  if (!ConfigGyroDlpfBandwidth(GYRO_DLPF_BANDWIDTH_119HZ)) {
    Serial.println("12");
    return false;
  }
  /* Set the SRD to 0 by default */
  if (!ConfigSrd(9)) {
    Serial.println("13");
    return false;
  }
// debug();
  return true;
}


void Icm20649::debug() {
  Serial.print("GYRO_SMPLRT_DIV: ");
  ReadRegisters(GYRO_SMPLRT_DIV, 1, data_buf_); Serial.println(data_buf_[0], BIN);
  Serial.print("GYRO_CONFIG_1: ");
  ReadRegisters(GYRO_CONFIG_1, 1, data_buf_); Serial.println(data_buf_[0], BIN);
  Serial.print("GYRO_CONFIG_2: ");
  ReadRegisters(GYRO_CONFIG_2, 1, data_buf_); Serial.println(data_buf_[0], BIN);
  Serial.print("ACCEL_SMPLRT_DIV_1: ");
  ReadRegisters(ACCEL_SMPLRT_DIV_1, 1, data_buf_); Serial.println(data_buf_[0], BIN);  
  Serial.print("ACCEL_SMPLRT_DIV_2: ");
  ReadRegisters(ACCEL_SMPLRT_DIV_2, 1, data_buf_); Serial.println(data_buf_[0], BIN);  
  Serial.print("ACCEL_CONFIG: ");
  ReadRegisters(ACCEL_CONFIG, 1, data_buf_); Serial.println(data_buf_[0], BIN);  
  Serial.print("ACCEL_CONFIG_2: ");
  ReadRegisters(ACCEL_CONFIG_2, 1, data_buf_); Serial.println(data_buf_[0], BIN);  
}


bool Icm20649::EnableDrdyInt() {
  spi_clock_ = SPI_CFG_CLOCK_;
    /* Set to Bank 0 */
  if(!setBank(0)) {
	return false;
  }
  //if (!WriteRegister(INT_PIN_CFG, INT_PULSE_50US_)) {
  //  return false;
  //}
  if (!WriteRegister(INT_ENABLE_1, 0x01)) {
    return false;
  }
  return true;
}
bool Icm20649::DisableDrdyInt() {
  spi_clock_ = SPI_CFG_CLOCK_;
    /* Set to Bank 0 */
  if(!setBank(0)) {
	return false;
  }
  if (!WriteRegister(INT_ENABLE_1, 0x00)) {
    return false;
  }
  return true;
}

bool Icm20649::clearInterrupts() {
	uint8_t regVal[1];
	setBank(0);
	ReadRegisters(INT_STATUS, sizeof(regVal), regVal);
	ReadRegisters(INT_STATUS_1, sizeof(regVal), regVal);
	return true;
}

bool Icm20649::ConfigAccelRange(const AccelRange range) {
  spi_clock_ = SPI_CFG_CLOCK_;
   if(!setBank(2)) {
	return false;
  }
  /* Check input is valid and set requested range and scale */
  switch (range) {
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
    case ACCEL_RANGE_30G: {
      requested_accel_range_ = range;
      requested_accel_scale_ = 30.0f / 32767.5f;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try setting the requested range */
  ReadRegisters(ACCEL_CONFIG, 1, data_buf_);
  data_buf_[0] &= ~(0x06);
  data_buf_[0] |= (requested_accel_range_ << 1);
  if (!WriteRegister(ACCEL_CONFIG, data_buf_[0] )) {
    return false;
  }
  /* Update stored range and scale */
  accel_range_ = requested_accel_range_;
  accel_scale_ = requested_accel_scale_;
  return true;
}
bool Icm20649::ConfigGyroRange(const GyroRange range) {
  spi_clock_ = SPI_CFG_CLOCK_;
   if(!setBank(2)) {
	return false;
  }
  /* Check input is valid and set requested range and scale */
  switch (range) {
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
    case GYRO_RANGE_4000DPS: {
      requested_gyro_range_ = range;
      requested_gyro_scale_ = 4000.0f / 32767.5f;
      break;
    }
    default: {
      return false;
    }
  }
  /* Try setting the requested range */
  
  ReadRegisters(GYRO_CONFIG_1, 1, data_buf_);
  data_buf_[0] &= ~(0x06);
  data_buf_[0] |= (requested_accel_range_ << 1);
  if (!WriteRegister(GYRO_CONFIG_1, data_buf_[0])) {
    return false;
  }
  /* Update stored range and scale */
  gyro_range_ = requested_gyro_range_;
  gyro_scale_ = requested_gyro_scale_;
  return true;
}

bool Icm20649::ConfigSrd(const uint8_t srd) {
  spi_clock_ = SPI_CFG_CLOCK_;
  if(!setBank(2)) {
	return false;
  }
  /* Changing the SRD to allow us to set the magnetometer successfully */
  uint8_t div1 = (srd << 8);
  uint8_t div2 = (srd & 0xFF);
  if (!WriteRegister(ACCEL_SMPLRT_DIV_1, (uint8_t)(srd<<8))) {
    return false;
  }
  if (!WriteRegister(ACCEL_SMPLRT_DIV_2, (uint8_t)(srd & 0xFF))) {
    return false;
  }
  if(!setBank(2)) {
	return false;
  }
  /* Set the IMU Accel sample rate */
  if (!WriteRegister(ACCEL_SMPLRT_DIV_1, div1<<8)) {
    return false;
  }
  if (!WriteRegister(ACCEL_SMPLRT_DIV_2, div2 & 0xFF)) {
    return false;
  }
  if (!WriteRegister(GYRO_SMPLRT_DIV, srd)) {
    return false;
  }  
  srd_ = srd;
  return true;
}

bool Icm20649::ConfigAccelDlpfBandwidth(const AccelDlpfBandwidth dlpf) {
  spi_clock_ = SPI_CFG_CLOCK_;
  if(!setBank(2)) {
    return false;
  }
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
  ReadRegisters(ACCEL_CONFIG, 1, data_buf_);
  if(dlpf == ACCEL_DLPF_BANDWIDTH_OFF){
    data_buf_[0] &= 0xFE;
    WriteRegister(ACCEL_CONFIG, data_buf_[0]);
    return true;
  } else {
    data_buf_[0] |= 0x01;
    data_buf_[0] &= 0xC7;
    data_buf_[0] |= (accel_requested_dlpf_<< 3);
  }
  if (!WriteRegister(ACCEL_CONFIG, data_buf_[0])) {
    return false;
  }

  /* Update stored dlpf */
  accel_dlpf_bandwidth_ = accel_requested_dlpf_;
  return true;
}

bool Icm20649::ConfigGyroDlpfBandwidth(const GyroDlpfBandwidth dlpf) {
  spi_clock_ = SPI_CFG_CLOCK_;
  if(!setBank(2)) {
    return false;
  }
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
  /* Try setting the dlpf */
  /* first set FCHOICE to 1 to enable dlpf */
    if (!WriteRegister(GYRO_CONFIG_1, 0x01)) {
    return false;
  }
  /* second change dlpf */
  ReadRegisters(GYRO_CONFIG_1, sizeof(data_buf_), data_buf_);
  if(dlpf == GYRO_DLPF_BANDWIDTH_OFF){
    data_buf_[0] &= 0xFE;
    WriteRegister(GYRO_CONFIG_1, data_buf_[0]);
    return true;
  } else {
    data_buf_[0] |= 0x01;
    data_buf_[0] &= 0xC7;
    data_buf_[0] |= (gyro_requested_dlpf_<< 3);
  }
  if (!WriteRegister(GYRO_CONFIG_1, data_buf_[0])) {
    return false;
  }

  /* Update stored dlpf */
  gyro_dlpf_bandwidth_ = gyro_requested_dlpf_;
  return true;
}

void Icm20649::Reset() {
  spi_clock_ = SPI_CFG_CLOCK_;
  /* Reset the MPU9250 */
  setBank(0);
  WriteRegister(PWR_MGMT_1, H_RESET_);
  /* Wait for MPU-9250 to come back up */
  delay(1);
}
bool Icm20649::Read() {
  spi_clock_ = SPI_READ_CLOCK_;
  setBank(0);
  /* Reset the new data flags */
  new_imu_data_ = false;
  /* Read the data registers */
  if (!ReadRegisters(INT_STATUS_1, 1, data_buf_)) {
    return false;
  }
  /* Check if data is ready */
  new_imu_data_ = (data_buf_[0] & RAW_DATA_RDY_INT_);
  if (!new_imu_data_) {
    return false;
  }
  ReadRegisters(ACCEL_XOUT_H, 14, data_buf_);
  /* Unpack the buffer */
  accel_cnts_[0] = static_cast<int16_t>(data_buf_[0])  << 8 | data_buf_[1];
  accel_cnts_[1] = static_cast<int16_t>(data_buf_[2])  << 8 | data_buf_[3];
  accel_cnts_[2] = static_cast<int16_t>(data_buf_[4])  << 8 | data_buf_[5];
  gyro_cnts_[0] =  static_cast<int16_t>(data_buf_[6])  << 8 | data_buf_[7];
  gyro_cnts_[1] =  static_cast<int16_t>(data_buf_[8])  << 8 | data_buf_[9];
  gyro_cnts_[2] =  static_cast<int16_t>(data_buf_[10]) << 8 | data_buf_[11];
  temp_cnts_ =     static_cast<int16_t>(data_buf_[12])  << 8 | data_buf_[13];
  
  /* Convert to float values and rotate the accel / gyro axis */
  accel_[0] = static_cast<float>(accel_cnts_[0]) * accel_scale_ * G_MPS2_;
  accel_[1] = static_cast<float>(accel_cnts_[1]) * accel_scale_ * -1.0f * G_MPS2_;
  accel_[2] = static_cast<float>(accel_cnts_[2]) * accel_scale_ * -1.0f * G_MPS2_;
  temp_ = (static_cast<float>(temp_cnts_) - 21.0f) / TEMP_SCALE_ + 21.0f;
  gyro_[0] = static_cast<float>(gyro_cnts_[0]) * gyro_scale_ * DEG2RAD_;
  gyro_[1] = static_cast<float>(gyro_cnts_[1]) * gyro_scale_ * -1.0f * DEG2RAD_;
  gyro_[2] = static_cast<float>(gyro_cnts_[2]) * gyro_scale_ * -1.0f * DEG2RAD_;

  return true;
}


bool Icm20649::setBank(uint8_t bank) {
	if(bank != currentBank_) {
		currentBank_ = bank;
		return WriteRegister(REG_BANK_SEL, bank << 4);
	}
	return 1;
}

bool Icm20649::WriteRegister(const uint8_t reg, const uint8_t data) {
  return imu_.WriteRegister(reg, data, spi_clock_);
}
bool Icm20649::ReadRegisters(const uint8_t reg, const uint8_t count,
                            uint8_t * const data) {
  return imu_.ReadRegisters(reg, count, spi_clock_, data);
}

}  // namespace bfs
