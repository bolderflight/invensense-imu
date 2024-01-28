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

#ifndef INVENSENSE_IMU_SRC_AK8963_H_  // NOLINT
#define INVENSENSE_IMU_SRC_AK8963_H_

#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif

namespace bfs {

class Ak8963 {
 public:
  enum MeasRate {
    MEAS_RATE_SINGLE,
    MEAS_RATE_8HZ,
    MEAS_RATE_100HZ
  };
  Ak8963() {}
  Ak8963(TwoWire *i2c) : i2c_(i2c) {}
  void Config(TwoWire *i2c);
  bool Begin();
  bool ConfigMeasRate(const MeasRate rate);
  MeasRate meas_rate() const {return meas_rate_;}
  bool Read();
  inline bool new_mag_data() const {return new_mag_data_;}
  inline float mag_x_ut() const {return mag_[0];}
  inline float mag_y_ut() const {return mag_[1];}
  inline float mag_z_ut() const {return mag_[2];}

 private:
  TwoWire *i2c_;
  static constexpr uint8_t dev_ = 0x0C;
  /* Config */
  MeasRate meas_rate_;
  float mag_scale_[3];
  static constexpr uint8_t WHOAMI_AK8963_ = 0x48;
  /* Data */
  size_t bytes_rx_;
  bool new_mag_data_;
  uint8_t buf_[7];
  int16_t mag_cnts_[3];
  float mag_[3];
  /* Registers */
  static constexpr uint8_t WHOAMI_ = 0x00;
  static constexpr uint8_t ST1_ = 0x02;
  static constexpr uint8_t ST1_DRDY_ = 0x01;
  static constexpr uint8_t ST1_DOR_ = 0x02;
  static constexpr uint8_t HXL_ = 0x03;
  static constexpr uint8_t ST2_HOFL_ = 0x08;
  static constexpr uint8_t ST2_DERR_ = 0x04;
  static constexpr uint8_t CNTL1_ = 0x0A;
  static constexpr uint8_t CNTL1_PWR_DOWN_ = 0x00;
  static constexpr uint8_t CNTL1_SINGLE_MEAS_ = 0x01;
  static constexpr uint8_t CNTL1_CONT_MEAS1_ = 0x12;
  static constexpr uint8_t CNTL1_CONT_MEAS2_ = 0x16;
  static constexpr uint8_t CNTL1_FUSE_ROM_ = 0x0F;
  static constexpr uint8_t CNTL2_ = 0x0B;
  static constexpr uint8_t CNTL2_SRST_ = 0x01;
  static constexpr uint8_t ASA_ = 0x10;
  /* Utility functions */
  void WriteRegister(const uint8_t reg, const uint8_t data);
  bool ReadRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t * const data);
};

}  // namespace bfs

#endif  // INVENSENSE_IMU_SRC_AK8963_H_ NOLINT
