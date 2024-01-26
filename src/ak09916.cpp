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

#include "ak09916.h"  // NOLINT
#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif

namespace bfs {

void Ak09916::Config(TwoWire *i2c) {
  i2c_ = i2c;
}
bool Ak09916::Begin() {
  delay(100);
  /* Check WHO AM I */
  if (!ReadRegisters(WIA2_, 1, buf_)) {
    return false;
  }
  if (buf_[0] != WHOAMI_AK09916_) {
    return false;
  }
  /* Soft reset */
  WriteRegister(CNTL3_, CNTL3_SRST_);
  delay(100);
  /* Configure the measurement rate */
  if (!ConfigMeasRate(MEAS_RATE_100HZ)) {
    return false;
  }
  return true;
}
bool Ak09916::ConfigMeasRate(const MeasRate rate) {
  switch (rate) {
    case MEAS_RATE_10HZ: {
      WriteRegister(CNTL2_, CNTL2_CONT_MEAS_MODE1_);
      if (!ReadRegisters(CNTL2_, 1, buf_)) {
        return false;
      }
      if (buf_[0] != CNTL2_CONT_MEAS_MODE1_) {
        return false;
      }
      return true;
    }
    case MEAS_RATE_20HZ: {
      WriteRegister(CNTL2_, CNTL2_CONT_MEAS_MODE2_);
      if (!ReadRegisters(CNTL2_, 1, buf_)) {
        return false;
      }
      if (buf_[0] != CNTL2_CONT_MEAS_MODE2_) {
        return false;
      }
      return true;
    }
    case MEAS_RATE_50HZ: {
      WriteRegister(CNTL2_, CNTL2_CONT_MEAS_MODE3_);
      if (!ReadRegisters(CNTL2_, 1, buf_)) {
        return false;
      }
      if (buf_[0] != CNTL2_CONT_MEAS_MODE3_) {
        return false;
      }
      return true;
    }
    case MEAS_RATE_100HZ: {
      WriteRegister(CNTL2_, CNTL2_CONT_MEAS_MODE4_);
      if (!ReadRegisters(CNTL2_, 1, buf_)) {
        return false;
      }
      if (buf_[0] != CNTL2_CONT_MEAS_MODE4_) {
        return false;
      }
      return true;
    }
    default: {
      return false;
    }
  }
}
bool Ak09916::Read1() {
  if (!ReadRegisters(ST1_, 9, buf_)) {
    return false;
  }
  new_mag_data_ = (buf_[0] & ST1_DRDY_);
  if (new_mag_data_) {
    mag_cnts_[0] =   static_cast<int16_t>(buf_[2]) << 8 | buf_[1];
    mag_cnts_[1] =   static_cast<int16_t>(buf_[4]) << 8 | buf_[3];
    mag_cnts_[2] =   static_cast<int16_t>(buf_[6]) << 8 | buf_[5];
    mag_sensor_overflow_ = (buf_[8] & ST2_HOFL_);
    if (mag_sensor_overflow_) {
      new_mag_data_ = false;
      return false;
    } else {
      mag_[0] =   -1.0f * (static_cast<float>(mag_cnts_[1]) * MAG_SCALE_);
      mag_[1] =   static_cast<float>(mag_cnts_[0]) * MAG_SCALE_;
      mag_[2] =   static_cast<float>(mag_cnts_[2]) * MAG_SCALE_;
      return true;
    }
  }
  return false;
}
bool Ak09916::Read2() {
  new_mag_data_ = false;
  if (!ReadRegisters(ST1_, 1, buf_)) {
    return false;
  }
  if (buf_[0] & ST1_DRDY_) {
    if (!ReadRegisters(HXL_, 8, buf_)) {
      return false;
    }
    mag_cnts_[0] =   static_cast<int16_t>(buf_[1]) << 8 | buf_[0];
    mag_cnts_[1] =   static_cast<int16_t>(buf_[3]) << 8 | buf_[2];
    mag_cnts_[2] =   static_cast<int16_t>(buf_[5]) << 8 | buf_[4];
    mag_sensor_overflow_ = (buf_[7] & ST2_HOFL_);
    if (mag_sensor_overflow_) {
      return false;
    }
    mag_[0] =   -1.0f * (static_cast<float>(mag_cnts_[1]) * MAG_SCALE_);
    mag_[1] =   static_cast<float>(mag_cnts_[0]) * MAG_SCALE_;
    mag_[2] =   static_cast<float>(mag_cnts_[2]) * MAG_SCALE_;
    new_mag_data_ = true;
    return true;
  }
  return false;
}
void Ak09916::WriteRegister(const uint8_t reg, const uint8_t data) {
  i2c_->beginTransmission(dev_);
  i2c_->write(reg);
  i2c_->write(data);
  i2c_->endTransmission();
}
bool Ak09916::ReadRegisters(const uint8_t reg, const uint8_t count,
                            uint8_t * const data) {
  i2c_->beginTransmission(dev_);
  i2c_->write(reg);
  i2c_->endTransmission(false);
  bytes_rx_ = i2c_->requestFrom(static_cast<uint8_t>(dev_), count);
  if (bytes_rx_ == count) {
    for (size_t i = 0; i < count; i++) {
      data[i] = i2c_->read();
    }
    return true;
  } else {
    return false;
  }
}

}  // namespace bfs
