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

#ifndef INVENSENSE_IMU_SRC_MPU6500_H_  // NOLINT
#define INVENSENSE_IMU_SRC_MPU6500_H_

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

class Mpu6500 {
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
  Mpu6500() {}
  Mpu6500(TwoWire *i2c, const I2cAddr addr) :
          imu_(i2c, static_cast<uint8_t>(addr)) {}
  Mpu6500(SPIClass *spi, const uint8_t cs) :
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
  bool Read();
  inline bool new_imu_data() const {return new_imu_data_;}
  inline float accel_x_mps2() const {return accel_[0];}
  inline float accel_y_mps2() const {return accel_[1];}
  inline float accel_z_mps2() const {return accel_[2];}
  inline float gyro_x_radps() const {return gyro_[0];}
  inline float gyro_y_radps() const {return gyro_[1];}
  inline float gyro_z_radps() const {return gyro_[2];}
  inline float die_temp_c() const {return temp_;}

 private:
  InvensenseImu imu_;
  int32_t spi_clock_;
  /*
  * MPU-6500 supports an SPI clock of 1 MHz for config and 20 MHz for reading
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
  uint8_t who_am_i_;
  static constexpr uint8_t WHOAMI_MPU6500_ = 0x70;
  /* Data */
  static constexpr float G_MPS2_ = 9.80665f;
  static constexpr float DEG2RAD_ = 3.14159265358979323846264338327950288f /
                                    180.0f;
  bool new_imu_data_;
  uint8_t data_buf_[15];
  int16_t accel_cnts_[3], gyro_cnts_[3], temp_cnts_;
  float accel_[3], gyro_[3];
  float temp_;
  /* Registers */
  static constexpr uint8_t PWR_MGMNT_1_ = 0x6B;
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
  /* Utility functions */
  bool WriteRegister(const uint8_t reg, const uint8_t data);
  bool ReadRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t * const data);
};

}  // namespace bfs

#endif  // INVENSENSE_IMU_SRC_MPU6500_H_ NOLINT
