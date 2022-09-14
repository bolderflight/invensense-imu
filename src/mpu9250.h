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

#ifndef INVENSENSE_IMU_SRC_MPU9250_H_  // NOLINT
#define INVENSENSE_IMU_SRC_MPU9250_H_

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

class Mpu9250 {
 public:
  /* Sensor and filter settings */
  enum I2cAddr : uint8_t {
    I2C_ADDR_PRIM = 0x68,
    I2C_ADDR_SEC = 0x69
  };
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
  enum WomRate : int8_t {
    WOM_RATE_0_24HZ = 0x00,
    WOM_RATE_0_49HZ = 0x01,
    WOM_RATE_0_98HZ = 0x02,
    WOM_RATE_1_95HZ = 0x03,
    WOM_RATE_3_91HZ = 0x04,
    WOM_RATE_7_81HZ = 0x05,
    WOM_RATE_15_63HZ = 0x06,
    WOM_RATE_31_25HZ = 0x07,
    WOM_RATE_62_50HZ = 0x08,
    WOM_RATE_125HZ = 0x09,
    WOM_RATE_250HZ = 0x0A,
    WOM_RATE_500HZ = 0x0B
  };
  Mpu9250() {}
  Mpu9250(TwoWire *i2c, const I2cAddr addr) :
          imu_(i2c, static_cast<uint8_t>(addr)) {}
  Mpu9250(SPIClass *spi, const uint8_t cs) :
          imu_(spi, cs) {}
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
  bool ConfigDlpfBandwidth(const DlpfBandwidth dlpf);
  inline DlpfBandwidth dlpf_bandwidth() const {return dlpf_bandwidth_;}
  bool EnableWom(int16_t threshold_mg, const WomRate wom_rate);
  void Reset();
  bool Read();
  inline bool new_imu_data() const {return new_imu_data_;}
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
  inline float die_temp_c() const {return temp_;}

 private:
  InvensenseImu imu_;
  int32_t spi_clock_;
  /*
  * MPU-9250 supports an SPI clock of 1 MHz for config and 20 MHz for reading
  * data; however, in testing we found that 20 MHz was sometimes too fast and
  * scaled this down to 15 MHz, which consistently worked well.
  */
  static constexpr int32_t SPI_CFG_CLOCK_ = 1000000;
  static constexpr int32_t SPI_READ_CLOCK_ = 15000000;
  /* Configuration */
  AccelRange accel_range_, requested_accel_range_;
  GyroRange gyro_range_, requested_gyro_range_;
  DlpfBandwidth dlpf_bandwidth_, requested_dlpf_;
  float accel_scale_, requested_accel_scale_;
  float gyro_scale_, requested_gyro_scale_;
  uint8_t srd_;
  static constexpr float TEMP_SCALE_ = 333.87f;
  uint8_t asa_buff_[3];
  float mag_scale_[3];
  uint8_t who_am_i_;
  static constexpr uint8_t WHOAMI_MPU9250_ = 0x71;
  static constexpr uint8_t WHOAMI_MPU9255_ = 0x73;
  static constexpr uint8_t WHOAMI_AK8963_ = 0x48;
  /* Data */
  static constexpr float G_MPS2_ = 9.80665f;
  static constexpr float DEG2RAD_ = 3.14159265358979323846264338327950288f /
                                    180.0f;
  bool new_imu_data_, new_mag_data_;
  bool mag_sensor_overflow_;
  uint8_t mag_data_[8];
  uint8_t data_buf_[23];
  int16_t accel_cnts_[3], gyro_cnts_[3], temp_cnts_, mag_cnts_[3];
  float accel_[3], gyro_[3], mag_[3];
  float temp_;
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
  /* Needed for WOM */
  static constexpr uint8_t INT_WOM_EN_ = 0x40;
  static constexpr uint8_t PWR_MGMNT_2_ = 0x6C;
  static constexpr uint8_t DISABLE_GYRO_ = 0x07;
  static constexpr uint8_t MOT_DETECT_CTRL_ = 0x69;
  static constexpr uint8_t ACCEL_INTEL_EN_ = 0x80;
  static constexpr uint8_t ACCEL_INTEL_MODE_ = 0x40;
  static constexpr uint8_t LP_ACCEL_ODR_ = 0x1E;
  static constexpr uint8_t WOM_THR_ = 0x1F;
  static constexpr uint8_t PWR_CYCLE_WOM_ = 0x20;
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
  bool WriteRegister(const uint8_t reg, const uint8_t data);
  bool ReadRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t * const data);
  bool WriteAk8963Register(const uint8_t reg, const uint8_t data);
  bool ReadAk8963Registers(const uint8_t reg, const uint8_t count,
                           uint8_t * const data);
};

}  // namespace bfs

#endif  // INVENSENSE_IMU_SRC_MPU9250_H_ NOLINT
