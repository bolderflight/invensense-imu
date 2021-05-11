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

#ifndef INCLUDE_MPU9250_MPU9250_H_
#define INCLUDE_MPU9250_MPU9250_H_

#include "Eigen/Core"
#include "Eigen/Dense"
#include "core/core.h"
#include "imu/imu.h"
#include "statistics/statistics.h"

namespace bfs {

class Mpu9250 {
 public:
  bool Init(const ImuConfig &ref);
  bool Read(ImuData * const ptr);

 private:
  /* Sensor and filter settings */
  enum DlpfBandwidth : int8_t {
    DLPF_BANDWIDTH_184HZ = 0x01,
    DLPF_BANDWIDTH_92HZ = 0x02,
    DLPF_BANDWIDTH_41HZ = 0x03,
    DLPF_BANDWIDTH_20HZ = 0x04,
    DLPF_BANDWIDTH_10HZ = 0x05,
    DLPF_BANDWIDTH_5HZ = 0x06
  };
  enum AccelRange : int8_t {
    ACCEL_RANGE_2G = 0x00,
    ACCEL_RANGE_4G = 0x08,
    ACCEL_RANGE_8G = 0x10,
    ACCEL_RANGE_16G = 0x18
  };
  enum GyroRange : int8_t {
    GYRO_RANGE_250DPS = 0x00,
    GYRO_RANGE_500DPS = 0x08,
    GYRO_RANGE_1000DPS = 0x10,
    GYRO_RANGE_2000DPS = 0x18
  };
  /* Communications interface */
  enum Interface : int8_t {
    SPI,
    I2C
  };
  Interface iface_;
  TwoWire *i2c_;
  SPIClass *spi_;
  int32_t spi_clock_;
  static constexpr uint8_t SPI_READ_ = 0x80;
  /* Configuration */
  ImuConfig config_;
  AccelRange accel_range_, requested_accel_range_;
  GyroRange gyro_range_, requested_gyro_range_;
  float accel_scale_, requested_accel_scale_;
  float gyro_scale_, requested_gyro_scale_;
  DlpfBandwidth dlpf_bandwidth_, requested_dlpf_;
  float mag_scale_[3];
  static constexpr float temp_scale_ = 333.87f;
  uint8_t srd_;
  uint8_t who_am_i_;
  uint8_t asa_buff_[3];
  static constexpr uint8_t WHOAMI_MPU9250_ = 0x71;
  static constexpr uint8_t WHOAMI_MPU9255_ = 0x73;
  static constexpr uint8_t WHOAMI_AK8963_ = 0x48;
  /* Gyro bias removal */
  elapsedMillis time_ms_;
  static constexpr int16_t INIT_TIME_MS_ = 2000;
  bfs::RunningStats<float> gx_, gy_, gz_;
  Eigen::Vector3f gyro_bias_radps_ = Eigen::Vector3f::Zero();
  /* Accel bias and scale factor */
  Eigen::Vector3f accel_bias_mps2_;
  Eigen::Matrix3f accel_scale_factor_;
  /* Mag bias and scale factor */
  Eigen::Vector3f mag_bias_ut_;
  /* Rotation */
  Eigen::Matrix3f rotation_;
  Eigen::Matrix3f mag_scale_factor_;
  /* Health determination */
  int16_t imu_health_period_ms_;
  int16_t mag_health_period_ms_;
  elapsedMillis imu_health_timer_ms_;
  elapsedMillis mag_health_timer_ms_;
  /* Data */
  uint8_t mag_data_[8];
  uint8_t data_buf_[23];
  int16_t accel_cnts_[3], gyro_cnts_[3], temp_cnts_, mag_cnts_[3];
  bool new_imu_data_, new_mag_data_;
  bool mag_sensor_overflow_;
  Eigen::Vector3f accel_, gyro_, mag_;
  Eigen::Vector3f accel_mps2_, gyro_radps_, mag_ut_; 
  float die_temperature_c_;
  /* Registers */
  static constexpr uint8_t PWR_MGMNT_1_ = 0x6B;
  static constexpr uint8_t H_RESET_ = 0x80;
  static constexpr uint8_t CLKSEL_PLL_ = 0x01;
  static constexpr uint8_t WHOAMI_ = 0x75;
  static constexpr uint8_t ACCEL_CONFIG_ = 0x1C;
  static constexpr uint8_t GYRO_CONFIG_ = 0x1B;
  static constexpr uint8_t ACCEL_CONFIG2_ = 0x1D;
  static constexpr uint8_t CONFIG_ = 0x1A;
  static constexpr uint8_t SMPLRT_DIV_ = 0x19;
  static constexpr uint8_t INT_PIN_CFG_ = 0x37;
  static constexpr uint8_t INT_ENABLE_ = 0x38;
  static constexpr uint8_t INT_DISABLE_ = 0x00;
  static constexpr uint8_t INT_PULSE_50US_ = 0x00;
  static constexpr uint8_t INT_RAW_RDY_EN_ = 0x01;
  static constexpr uint8_t INT_STATUS_ = 0x3A;
  static constexpr uint8_t RAW_DATA_RDY_INT_ = 0x01;
  static constexpr uint8_t USER_CTRL_ = 0x6A;
  static constexpr uint8_t I2C_MST_EN_ = 0x20;
  static constexpr uint8_t I2C_MST_CLK_ = 0x0D;
  static constexpr uint8_t I2C_MST_CTRL_ = 0x24;
  static constexpr uint8_t I2C_SLV0_ADDR_ = 0x25;
  static constexpr uint8_t I2C_SLV0_REG_ = 0x26;
  static constexpr uint8_t I2C_SLV0_CTRL_ = 0x27;
  static constexpr uint8_t I2C_SLV0_DO_ = 0x63;
  static constexpr uint8_t I2C_READ_FLAG_ = 0x80;
  static constexpr uint8_t I2C_SLV0_EN_ = 0x80;
  static constexpr uint8_t EXT_SENS_DATA_00_ = 0x49;
  /* AK8963 registers */
  static constexpr uint8_t AK8963_I2C_ADDR_ = 0x0C;
  static constexpr uint8_t AK8963_ST1_ = 0x02;
  static constexpr uint8_t AK8963_DATA_RDY_INT_ = 0x01;
  static constexpr uint8_t AK8963_HXL_ = 0x03;
  static constexpr uint8_t AK8963_CNTL1_ = 0x0A;
  static constexpr uint8_t AK8963_PWR_DOWN_ = 0x00;
  static constexpr uint8_t AK8963_CNT_MEAS1_ = 0x12;
  static constexpr uint8_t AK8963_CNT_MEAS2_ = 0x16;
  static constexpr uint8_t AK8963_FUSE_ROM_ = 0x0F;
  static constexpr uint8_t AK8963_CNTL2_ = 0x0B;
  static constexpr uint8_t AK8963_RESET_ = 0x01;
  static constexpr uint8_t AK8963_ASA_ = 0x10;
  static constexpr uint8_t AK8963_WHOAMI_ = 0x00;
  static constexpr uint8_t AK8963_HOFL_ = 0x08;
  /* Utility functions */
  bool Begin();
  bool EnableDrdyInt();
  bool DisableDrdyInt();
  bool ConfigAccelRange(const AccelRange range);
  bool ConfigGyroRange(const GyroRange range);
  bool ConfigSrd(const uint8_t srd);
  bool ConfigDlpf(const DlpfBandwidth dlpf);
  bool ReadImu();
  bool WriteRegister(uint8_t reg, uint8_t data);
  bool ReadRegisters(uint8_t reg, uint8_t count, uint8_t *data);
  bool WriteAk8963Register(uint8_t reg, uint8_t data);
  bool ReadAk8963Registers(uint8_t reg, uint8_t count, uint8_t *data);
};

/* Checking conformance to IMU interface */
static_assert(Imu<Mpu9250>, "MPU-9250 does not conform to IMU interface");

}  // namespace bfs

#endif  // INCLUDE_MPU9250_MPU9250_H_
