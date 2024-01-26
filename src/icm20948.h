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

#ifndef INVENSENSE_IMU_SRC_ICM20948_H_  // NOLINT
#define INVENSENSE_IMU_SRC_ICM20948_H_

#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif
#include "invensense_imu.h"  // NOLINT

namespace bfs {

class Icm20948 {
 public:
  /* Sensor and filter settings */
  enum I2cAddr : uint8_t {
    I2C_ADDR_PRIM = 0x68,
    I2C_ADDR_SEC = 0x69
  };
  enum GyroDlpfBandwidth : int8_t {
    GYRO_DLPF_BANDWIDTH_196HZ = 0x00,
    GYRO_DLPF_BANDWIDTH_151HZ = 0x01,
    GYRO_DLPF_BANDWIDTH_119HZ = 0x02,
    GYRO_DLPF_BANDWIDTH_51HZ = 0x03,
    GYRO_DLPF_BANDWIDTH_23HZ = 0x04,
    GYRO_DLPF_BANDWIDTH_11HZ = 0x05,
    GYRO_DLPF_BANDWIDTH_5HZ = 0x06,
    GYRO_DLPF_BANDWIDTH_361HZ = 0x07,
  };
  enum AccelDlpfBandwidth : int8_t {
    ACCEL_DLPF_BANDWIDTH_246HZ = 0x01,
    ACCEL_DLPF_BANDWIDTH_111HZ = 0x02,
    ACCEL_DLPF_BANDWIDTH_50HZ = 0x03,
    ACCEL_DLPF_BANDWIDTH_23HZ = 0x04,
    ACCEL_DLPF_BANDWIDTH_11HZ = 0x05,
    ACCEL_DLPF_BANDWIDTH_5HZ = 0x06,
    ACCEL_DLPF_BANDWIDTH_473HZ = 0x07,
  };
  enum TempDlpfBandwidth : int8_t {
    TEMP_DLPF_BANDWIDTH_7932HZ = 0x00,
    TEMP_DLPF_BANDWIDTH_217HZ = 0x01,
    TEMP_DLPF_BANDWIDTH_123HZ = 0x02,
    TEMP_DLPF_BANDWIDTH_65HZ = 0x03,
    TEMP_DLPF_BANDWIDTH_34HZ = 0x04,
    TEMP_DLPF_BANDWIDTH_17HZ = 0x05,
    TEMP_DLPF_BANDWIDTH_8HZ = 0x06
  };
  enum AccelRange : int8_t {
    ACCEL_RANGE_2G	= 0x00,
    ACCEL_RANGE_4G	= 0x01,
    ACCEL_RANGE_8G = 0x02,
    ACCEL_RANGE_16G = 0x03
  };
  enum GyroRange : int8_t {
    GYRO_RANGE_250DPS	= 0x00,
    GYRO_RANGE_500DPS = 0x01,
    GYRO_RANGE_1000DPS = 0x02,
    GYRO_RANGE_2000DPS = 0x03
  };
  Icm20948() {}
  Icm20948(TwoWire *i2c, const I2cAddr addr) :
           imu_(i2c, static_cast<uint8_t>(addr)), iface_(I2C) {}
  Icm20948(SPIClass *spi, const uint8_t cs) :
           imu_(spi, cs), iface_(SPI) {}
  void Config(TwoWire *i2c, const I2cAddr addr);
  void Config(SPIClass *spi, const uint8_t cs);
  bool Begin();
  bool EnableDrdyInt();
  bool DisableDrdyInt();
  bool ConfigAccelRange(const AccelRange range);
  inline AccelRange accel_range() const {return accel_range_;}
  bool ConfigGyroRange(const GyroRange range);
  inline GyroRange gyro_range() const {return gyro_range_;}
  bool ConfigSrd(const uint8_t srd);
  inline uint8_t srd() const {return srd_;}
  bool ConfigAccelDlpfBandwidth(const AccelDlpfBandwidth dlpf);
  inline AccelDlpfBandwidth accel_dlpf_bandwidth() const {return accel_dlpf_bandwidth_;}
  bool ConfigGyroDlpfBandwidth(const GyroDlpfBandwidth dlpf);
  inline GyroDlpfBandwidth gyro_dlpf_bandwidth() const {return gyro_dlpf_bandwidth_;}
  bool ConfigTempDlpfBandwidth(const TempDlpfBandwidth dlpf);
  inline TempDlpfBandwidth temp_dlpf_bandwidth() const {return temp_dlpf_bandwidth_;}
  bool Read();
  inline bool new_imu_data() const {return new_imu_data_;}
  inline float die_temp_c() const {return temp_;}
  inline float accel_x_mps2() const {return accel_[0];}
  inline float accel_y_mps2() const {return accel_[1];}
  inline float accel_z_mps2() const {return accel_[2];}
  inline float gyro_x_radps() const {return gyro_[0];}
  inline float gyro_y_radps() const {return gyro_[1];}
  inline float gyro_z_radps() const {return gyro_[2];}
  inline bool new_mag_data() const {return new_mag_data_;}
  inline float mag_x_ut() const {return mag_[0];}
  inline float mag_y_ut() const {return mag_[1];}
  inline float mag_z_ut() const {return mag_[2];}

 private:
  InvensenseImu imu_;
  enum Interface : int8_t {
    SPI,
    I2C
  };
  Interface iface_;
  static constexpr int32_t SPI_CLOCK_ = 5000000;
  uint8_t current_bank_;
  /* Configuration */
  AccelRange accel_range_, requested_accel_range_;
  GyroRange gyro_range_, requested_gyro_range_;
  AccelDlpfBandwidth accel_dlpf_bandwidth_, accel_requested_dlpf_;
  GyroDlpfBandwidth gyro_dlpf_bandwidth_, gyro_requested_dlpf_;
  TempDlpfBandwidth temp_dlpf_bandwidth_, temp_requested_dlpf_;
  float accel_scale_, requested_accel_scale_;
  float gyro_scale_, requested_gyro_scale_;
  uint8_t srd_;
  static constexpr float MAG_SCALE_ = 4912.0f / 32752.0f;
  static constexpr float TEMP_SCALE_ = 333.87f;
  uint8_t who_am_i_;
  static constexpr uint8_t WHOAMI_ICM20649_ = 0xEA;
  static constexpr uint8_t WHOAMI_AK09916_ = 0x09;
  /* Data */
  static constexpr float G_MPS2_ = 9.80665f;
  static constexpr float DEG2RAD_ = 3.14159265358979323846264338327950288f /
                                    180.0f;
  bool new_imu_data_;
  bool new_mag_data_;
  bool mag_sensor_overflow_;
  uint8_t mag_data_[9];
  uint8_t data_buf_[23];
  int16_t accel_cnts_[3], gyro_cnts_[3], temp_cnts_, mag_cnts_[3];
  float accel_[3], gyro_[3], mag_[3];
  float temp_;
  /* MASKS */
  static constexpr uint8_t RAW_DATA_RDY_INT_ = 0x01;
  /* Bank 0 */
  static constexpr uint8_t BANK_0_ = 0;
  static constexpr uint8_t WHO_AM_I_ = 0x00;
  static constexpr uint8_t USER_CTRL_ = 0x03;
  static constexpr uint8_t USER_CTRL_I2C_IF_DIS_ = 0x10;
  static constexpr uint8_t USER_CTRL_I2C_MST_EN_ = 0x20;
  static constexpr uint8_t PWR_MGMT_1_ = 0x06;
  static constexpr uint8_t PWR_MGMT_1_RESET_ = 0x80;
  static constexpr uint8_t PWR_MGMT_1_CLKSEL_AUTO_ = 0x01;
  static constexpr uint8_t INT_ENABLE_1_ = 0x11;
  static constexpr uint8_t INT_ENABLE_1_INT_ENABLE_ = 0x01;
  static constexpr uint8_t INT_ENABLE_1_INT_DISABLE_ = 0x00;
  static constexpr uint8_t INT_STATUS_1_ = 0x1A;
  static constexpr uint8_t INT_STATUS_1_RAW_DATA_RDY_INT_ = 0x01;
  static constexpr uint8_t ACCEL_XOUT_H = 0x2D;
  static constexpr uint8_t REG_BANK_SEL_ = 0x7F;
  static constexpr uint8_t EXT_SLV_SENS_DATA_00_ = 0x3B;
  /* Bank 2 */
  static constexpr uint8_t BANK_2_ = 2;
  static constexpr uint8_t GYRO_SMPLRT_DIV_ = 0x00;
  static constexpr uint8_t GYRO_CONFIG_1_ = 0x01;
  static constexpr uint8_t ALIGN_ODR_ = 0x01;
  static constexpr uint8_t ODR_ALIGN_EN_ = 0x09;
  static constexpr uint8_t ODR_ALIGN_EN_ALIGN_ENABLE_ = 0x01;
  static constexpr uint8_t ACCEL_SMPLRT_DIV_2_ = 0x11;
  static constexpr uint8_t ACCEL_CONFIG_ = 0x14;
  static constexpr uint8_t TEMP_CONFIG_ = 0x53;
  /* Bank 3 */
  static constexpr uint8_t BANK_3_ = 3;
  static constexpr uint8_t I2C_MST_CTRL_ = 0x01;
  static constexpr uint8_t I2C_MST_CTRL_400_KHZ_CLK_ = 0x07;
  static constexpr uint8_t I2C_SLV0_ADDR_ = 0x03;
  static constexpr uint8_t I2C_SLV0_ADDR_READ_ = 0x80;
  static constexpr uint8_t I2C_SLV0_REG_ = 0x04;
  static constexpr uint8_t I2C_SLV0_CTRL_ = 0x05;
  static constexpr uint8_t I2C_SLV0_CTRL_EN_ = 0x80;
  static constexpr uint8_t I2C_SLV0_D0_ = 0x06;
  /* AK09916 */
  static constexpr uint8_t AK09916_I2C_ADDR_ = 0x0C;  // I2C address
  static constexpr uint8_t AK09916_WIA2_ = 0x01;
  static constexpr uint8_t AK09916_CNTL2_ = 0x31;
  static constexpr uint8_t AK09916_CNTL2_PWR_DOWN_ = 0x00;
  static constexpr uint8_t AK09916_CNTL2_SINGLE_MEAS_ = 0x01;
  static constexpr uint8_t AK09916_CNTL2_CONT_MEAS_MODE1_ = 0x02;  // 10 Hz
  static constexpr uint8_t AK09916_CNTL2_CONT_MEAS_MODE2_ = 0x04;  // 20 Hz
  static constexpr uint8_t AK09916_CNTL2_CONT_MEAS_MODE3_ = 0x06;  // 50 Hz
  static constexpr uint8_t AK09916_CNTL2_CONT_MEAS_MODE4_ = 0x08;  // 100 Hz
  static constexpr uint8_t AK09916_ST1_ = 0x10;
  static constexpr uint8_t AK09916_ST1_DRDY_ = 0x01;
  static constexpr uint8_t AK09916_ST1_DOR_ = 0x02;
  static constexpr uint8_t AK09916_ST2_HOFL_ = 0x08;
  static constexpr uint8_t AK09916_CNTL3_ = 0x32;
  static constexpr uint8_t AK09916_CNTL3_SRST_ = 0x01;
  /* Utility functions */
  bool WriteRegister(const uint8_t bank, const uint8_t reg, const uint8_t data);
  bool ReadRegisters(const uint8_t bank, const uint8_t reg, const uint8_t count,
                     uint8_t * const data);
  bool WriteAk09916Register(const uint8_t reg, const uint8_t data);
  bool ReadAk09916Registers(const uint8_t reg, const uint8_t count,
                            uint8_t * const data);
};

}  // namespace bfs

#endif  // INVENSENSE_IMU_SRC_ICM20649_H_ NOLINT
