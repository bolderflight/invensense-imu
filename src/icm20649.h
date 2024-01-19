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

class Icm20649 {
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
    GYRO_DLPF_BANDWIDTH_OFF
  };
  enum AccelDlpfBandwidth : int8_t {
    ACCEL_DLPF_BANDWIDTH_246HZ = 0x01,
    ACCEL_DLPF_BANDWIDTH_111HZ = 0x02,
    ACCEL_DLPF_BANDWIDTH_50HZ = 0x03,
    ACCEL_DLPF_BANDWIDTH_23HZ = 0x04,
    ACCEL_DLPF_BANDWIDTH_11HZ = 0x05,
    ACCEL_DLPF_BANDWIDTH_5HZ = 0x06,
    ACCEL_DLPF_BANDWIDTH_473HZ = 0x07,
    ACCEL_DLPF_BANDWIDTH_OFF
  };
  enum AccelRange : int8_t {
    ACCEL_RANGE_4G	= 0x00,
    ACCEL_RANGE_8G	= 0x01,
    ACCEL_RANGE_16G = 0x02,
    ACCEL_RANGE_30G = 0x03
  };
  enum GyroRange : int8_t {
    GYRO_RANGE_500DPS	= 0x00,
    GYRO_RANGE_1000DPS	= 0x01,
    GYRO_RANGE_2000DPS	= 0x02,
    GYRO_RANGE_4000DPS	= 0x03
  };
  Icm20649() {}
  Icm20649(TwoWire *i2c, const I2cAddr addr) :
           imu_(i2c, static_cast<uint8_t>(addr)) {}
  Icm20649(SPIClass *spi, const uint8_t cs) :
           imu_(spi, cs) {}
  void Config(TwoWire *i2c, const I2cAddr addr);
  void Config(SPIClass *spi, const uint8_t cs);
  bool Begin();
  bool EnableDrdyInt();
  bool DisableDrdyInt();
  bool clearInterrupts();
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
  void Reset();
  bool Read();
  inline bool new_imu_data() const {return new_imu_data_;}
  inline float die_temp_c() const {return temp_;}
  inline float accel_x_mps2() const {return accel_[0];}
  inline float accel_y_mps2() const {return accel_[1];}
  inline float accel_z_mps2() const {return accel_[2];}
  inline float gyro_x_radps() const {return gyro_[0];}
  inline float gyro_y_radps() const {return gyro_[1];}
  inline float gyro_z_radps() const {return gyro_[2];}
  void debug();

 private:
  InvensenseImu imu_;
  int32_t spi_clock_;
  /*
  * MPU-9250 supports an SPI clock of 1 MHz for config and 20 MHz for reading
  * data; however, in testing we found that 20 MHz was sometimes too fast and
  * scaled this down to 15 MHz, which consistently worked well.
  */
  static constexpr int32_t SPI_CFG_CLOCK_ = 1000000;
  static constexpr int32_t SPI_READ_CLOCK_ = 7000000;
  /* Configuration */
  AccelRange accel_range_, requested_accel_range_;
  GyroRange gyro_range_, requested_gyro_range_;
  AccelDlpfBandwidth accel_dlpf_bandwidth_, accel_requested_dlpf_;
  GyroDlpfBandwidth gyro_dlpf_bandwidth_, gyro_requested_dlpf_;
  float accel_scale_, requested_accel_scale_;
  float gyro_scale_, requested_gyro_scale_;
  uint8_t srd_;
  static constexpr float TEMP_SCALE_ = 333.87f;
  uint8_t who_am_i_;
  static constexpr uint8_t WHOAMI_ICM20649_ = 0xE1;
  
  /* Data */
  static constexpr float G_MPS2_ = 9.80665f;
  static constexpr float DEG2RAD_ = 3.14159265358979323846264338327950288f /
                                    180.0f;
  bool new_imu_data_;
  uint8_t data_buf_[23];
  int16_t accel_cnts_[3], gyro_cnts_[3], temp_cnts_;
  float accel_[3], gyro_[3];
  float temp_;
  uint8_t currentBank_;
  /* Registers */
  static constexpr uint8_t CLKSEL_PLL_ = 0x01;
  static constexpr uint8_t REG_BANK_SEL = 0x7F;
  static constexpr uint8_t I2C_MST_EN_ = 0x20;
  //To achieve a targeted clock frequency of 400 kHz, MAX, it is recommended to set I2C_MST_CLK = 7 (345.6 kHz / 46.67% duty cycle).
  static constexpr uint8_t I2C_MST_CLK_ = 0x07; //was 0x0D
  static constexpr uint8_t H_RESET_ = 0x80;
  static constexpr uint8_t INT_PULSE_50US_ = 0x00;
  static constexpr uint8_t INT_RAW_RDY_EN_ = 0x01;
  static constexpr uint8_t INT_DISABLE_ = 0x00;
  static constexpr uint8_t I2C_READ_FLAG_ = 0x80;
  static constexpr uint8_t I2C_SLV0_EN_ = 0x80;
  static constexpr uint8_t RAW_DATA_RDY_INT_ = 0x01;
  /* Bank 0 */
  static constexpr uint8_t WHO_AM_I = 0x00;
  static constexpr uint8_t USER_CTRL = 0x03;
  static constexpr uint8_t LP_CONFIG = 0x05;
  static constexpr uint8_t PWR_MGMT_1 = 0x06;
  static constexpr uint8_t PWR_MGMT_2 = 0x07;
  static constexpr uint8_t INT_PIN_CFG = 0x0F;
  static constexpr uint8_t INT_ENABLE = 0x10;
  static constexpr uint8_t INT_ENABLE_1 = 0x11;
  static constexpr uint8_t INT_ENABLE_2 = 0x12; //FIFO_ovf
  static constexpr uint8_t INT_ENABLE_3 = 0x13; //FIFO_WM_EN
  static constexpr uint8_t I2C_MST_STATUS = 0x17;
  static constexpr uint8_t INT_STATUS = 0x19;
  static constexpr uint8_t INT_STATUS_1 = 0x1A;
  static constexpr uint8_t INT_STATUS_2 = 0x1B;
  static constexpr uint8_t INT_STATUS_3 = 0x1C;
  static constexpr uint8_t DELAY_TIMEH = 0x28;
  static constexpr uint8_t DELAY_TIMEL = 0x29;
  static constexpr uint8_t ACCEL_XOUT_H = 0x2D;
  static constexpr uint8_t ACCEL_XOUT_L = 0x2E;
  static constexpr uint8_t ACCEL_YOUT_H = 0x2F;
  static constexpr uint8_t ACCEL_YOUT_L = 0x30;
  static constexpr uint8_t ACCEL_ZOUT_H = 0x31;
  static constexpr uint8_t ACCEL_ZOUT_L = 0x32;
  static constexpr uint8_t GYRO_XOUT_H = 0x33;
  static constexpr uint8_t GYRO_XOUT_L = 0x34;
  static constexpr uint8_t GYRO_YOUT_H = 0x35;
  static constexpr uint8_t GYRO_YOUT_L = 0x36;
  static constexpr uint8_t GYRO_ZOUT_H = 0x37;
  static constexpr uint8_t GYRO_ZOUT_L = 0x38;
  static constexpr uint8_t TEMP_OUT_H = 0x39;
  static constexpr uint8_t TEMP_OUT_L = 0x3A;
  static constexpr uint8_t EXT_SLV_SENS_DATA_00 = 0x3B;
  static constexpr uint8_t EXT_SLV_SENS_DATA_01 = 0x3C;
  static constexpr uint8_t EXT_SLV_SENS_DATA_02 = 0x3D;
  static constexpr uint8_t EXT_SLV_SENS_DATA_03 = 0x3E;
  static constexpr uint8_t EXT_SLV_SENS_DATA_04 = 0x3F;
  static constexpr uint8_t EXT_SLV_SENS_DATA_05 = 0x40;
  static constexpr uint8_t EXT_SLV_SENS_DATA_06 = 0x41;
  static constexpr uint8_t EXT_SLV_SENS_DATA_07 = 0x42;
  static constexpr uint8_t EXT_SLV_SENS_DATA_08 = 0x43;
  static constexpr uint8_t EXT_SLV_SENS_DATA_09 = 0x44;
  static constexpr uint8_t EXT_SLV_SENS_DATA_10 = 0x45;
  static constexpr uint8_t EXT_SLV_SENS_DATA_11 = 0x46;
  static constexpr uint8_t EXT_SLV_SENS_DATA_12 = 0x47;
  static constexpr uint8_t EXT_SLV_SENS_DATA_13 = 0x48;
  static constexpr uint8_t EXT_SLV_SENS_DATA_14 = 0x49;
  static constexpr uint8_t EXT_SLV_SENS_DATA_15 = 0x4A;
  static constexpr uint8_t EXT_SLV_SENS_DATA_16 = 0x4B;
  static constexpr uint8_t EXT_SLV_SENS_DATA_17 = 0x4C;
  static constexpr uint8_t EXT_SLV_SENS_DATA_18 = 0x4D;
  static constexpr uint8_t EXT_SLV_SENS_DATA_19 = 0x4E;
  static constexpr uint8_t EXT_SLV_SENS_DATA_20 = 0x4F;
  static constexpr uint8_t EXT_SLV_SENS_DATA_21 = 0x50;
  static constexpr uint8_t EXT_SLV_SENS_DATA_22 = 0x51;
  static constexpr uint8_t EXT_SLV_SENS_DATA_23 = 0x52;
  static constexpr uint8_t FIFO_EN_1 = 0x66;
  static constexpr uint8_t FIFO_EN_2 = 0x67;
  static constexpr uint8_t FIFO_RST = 0x68;
  static constexpr uint8_t FIFO_MODE = 0x69;
  static constexpr uint8_t FIFO_COUNTH = 0x70;
  static constexpr uint8_t FIFO_COUNTL = 0x71;
  static constexpr uint8_t FIFO_R_W = 0x72;
  static constexpr uint8_t DATA_RDY_STATUS = 0x74;
  static constexpr uint8_t FIFO_CFG = 0x76;
  /* BANK 1 Register Map */
  static constexpr uint8_t SELF_TEST_X_GYRO = 0x02;
  static constexpr uint8_t SELF_TEST_Y_GYRO = 0x03;
  static constexpr uint8_t SELF_TEST_Z_GYRO = 0x04;
  static constexpr uint8_t SELF_TEST_X_ACCEL = 0x0E;
  static constexpr uint8_t SELF_TEST_Y_ACCEL = 0x0F;
  static constexpr uint8_t SELF_TEST_Z_ACCEL = 0x10;
  static constexpr uint8_t XA_OFFS_H = 0x14;
  static constexpr uint8_t XA_OFFS_L = 0x15;
  static constexpr uint8_t YA_OFFS_H = 0x17;
  static constexpr uint8_t YA_OFFS_L = 0x18;
  static constexpr uint8_t ZA_OFFS_H = 0x1A;
  static constexpr uint8_t ZA_OFFS_L = 0x1B;
  static constexpr uint8_t TIMEBASE_CORRECTION_PLL = 0x28;
  //REG_BANK_SEL = 0X7F;
  /* BANK 2 Register Map */
  static constexpr uint8_t GYRO_SMPLRT_DIV = 0x00;
  static constexpr uint8_t GYRO_CONFIG_1 = 0x01;
  static constexpr uint8_t GYRO_CONFIG_2 = 0x02;
  static constexpr uint8_t XG_OFFS_USRH = 0x03;
  static constexpr uint8_t XG_OFFS_USRL = 0x04;
  static constexpr uint8_t YG_OFFS_USRH = 0x05;
  static constexpr uint8_t YG_OFFS_USRL = 0x06;
  static constexpr uint8_t ZG_OFFS_USRH = 0x07;
  static constexpr uint8_t ZG_OFFS_USRL = 0x08;
  static constexpr uint8_t ODR_ALIGN_EN = 0x09;
  static constexpr uint8_t ACCEL_SMPLRT_DIV_1 = 0x10;
  static constexpr uint8_t ACCEL_SMPLRT_DIV_2 = 0x11;
  static constexpr uint8_t ACCEL_INTEL_CTRL = 0x12;
  static constexpr uint8_t ACCEL_WOM_THR = 0x13;
  static constexpr uint8_t ACCEL_CONFIG = 0x14;
  static constexpr uint8_t ACCEL_CONFIG_2 = 0x15;
  static constexpr uint8_t FSYNC_CONFIG = 0x52;
  static constexpr uint8_t TEMP_CONFIG = 0x53;
  static constexpr uint8_t MOD_CTRL_USR = 0x54;
  //REG_BANK_SEL = 0X7F;
  /* Bank 3 Register Map */
  static constexpr uint8_t I2C_MST_ODR_CONFIG = 0x00;
  static constexpr uint8_t I2C_MST_CTRL = 0x01;
  static constexpr uint8_t I2C_MST_DELAY_CTRL = 0x02;
  static constexpr uint8_t I2C_SLV0_ADDR = 0x03;
  static constexpr uint8_t I2C_SLV0_REG = 0x04;
  static constexpr uint8_t I2C_SLV0_CTRL = 0x05;
  static constexpr uint8_t I2C_SLV0_DO = 0x06;
  static constexpr uint8_t I2C_SLV1_ADDR = 0x07;
  static constexpr uint8_t I2C_SLV1_REG = 0x08;
  static constexpr uint8_t I2C_SLV1_CTRL = 0x09;
  static constexpr uint8_t I2C_SLV1_DO = 0x0A;
  static constexpr uint8_t I2C_SLV2_ADDR = 0x0B;
  static constexpr uint8_t I2C_SLV2_REG = 0x0C;
  static constexpr uint8_t I2C_SLV2_CTRL = 0x0D;
  static constexpr uint8_t I2C_SLV2_DO = 0x0E;
  static constexpr uint8_t I2C_SLV3_ADDR = 0x0F;
  static constexpr uint8_t I2C_SLV3_REG = 0x10;
  static constexpr uint8_t I2C_SLV3_CTRL = 0x11;
  static constexpr uint8_t I2C_SLV3_DO = 0x12;
  static constexpr uint8_t I2C_SLV4_ADDR = 0x13;
  static constexpr uint8_t I2C_SLV4_REG = 0x14;
  static constexpr uint8_t I2C_SLV4_CTRL = 0x15;
  static constexpr uint8_t I2C_SLV4_DO = 0x16;
  static constexpr uint8_t I2C_SLV4_DI = 0x17;

  /* Utility functions */
  bool WriteRegister(const uint8_t reg, const uint8_t data);
  bool ReadRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t * const data);
  bool setBank(uint8_t bank);
};

}  // namespace bfs

#endif  // INVENSENSE_IMU_SRC_ICM20649_H_ NOLINT
