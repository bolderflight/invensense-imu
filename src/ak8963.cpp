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

#include "ak8963.h"
#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif

namespace bfs {

void Ak8963::Config(TwoWire *i2c) {
  i2c_ = i2c;
}
bool Ak8963::Begin() {
  delay(100);
  /* Soft reset */
  WriteRegister(CNTL2_, CNTL2_SRST_);
  delay(100);
  /* Check WHO AM I */
  if (!ReadRegisters(WHOAMI_, 1, buf_)) {
    return false;
  }
  if (buf_[0] != WHOAMI_AK8963_) {
    return false;
  }
  /* Power down */
  WriteRegister(CNTL1_, CNTL1_PWR_DOWN_);
  delay(100);
  /* Enter fuse ROM mode */
  WriteRegister(CNTL1_, CNTL1_FUSE_ROM_);
  delay(100);
  /* Get fuse registers */
  if (!ReadRegisters(ASA_, 3, buf_)) {
    return false;
  }
  mag_scale_[0] = ((static_cast<float>(buf_[0]) - 128.0f)
    / 256.0f + 1.0f) * 4912.0f / 32760.0f;
  mag_scale_[1] = ((static_cast<float>(buf_[1]) - 128.0f)
    / 256.0f + 1.0f) * 4912.0f / 32760.0f;
  mag_scale_[2] = ((static_cast<float>(buf_[2]) - 128.0f)
    / 256.0f + 1.0f) * 4912.0f / 32760.0f;
  WriteRegister(CNTL1_, CNTL1_PWR_DOWN_);
  delay(100);
  /* Configure the measurement rate */
  if (!ConfigMeasRate(MEAS_RATE_100HZ)) {
    return false;
  }
  return true;
}
bool Ak8963::ConfigMeasRate(const MeasRate rate) {
  switch (rate) {
    case MEAS_RATE_SINGLE: {
      WriteRegister(CNTL1_, CNTL1_SINGLE_MEAS_);
      meas_rate_ = rate;
      return true;
    }
    case MEAS_RATE_8HZ: {
      WriteRegister(CNTL1_, CNTL1_CONT_MEAS1_);
      if (!ReadRegisters(CNTL1_, 1, buf_)) {
        return false;
      }
      if (buf_[0] != CNTL1_CONT_MEAS1_) {
        return false;
      }
      meas_rate_ = rate;
      return true;
    }
    case MEAS_RATE_100HZ: {
      WriteRegister(CNTL1_, CNTL1_CONT_MEAS2_);
      if (!ReadRegisters(CNTL1_, 1, buf_)) {
        return false;
      }
      if (buf_[0] != CNTL1_CONT_MEAS2_) {
        return false;
      }
      meas_rate_ = rate;
      return true;
    }
    default: {
      return false;
    }
  }
}
bool Ak8963::Read() {
  new_mag_data_ = false;
  if (!ReadRegisters(ST1_, 1, buf_)) {
    return false;
  }
  /* Check data ready */
  if (buf_[0] & ST1_DRDY_) {
    if (!ReadRegisters(HXL_, 7, buf_)) {
      return false;
    }
    /* Request another measurement */
    if (meas_rate_ == MEAS_RATE_SINGLE) {
      WriteRegister(CNTL1_, CNTL1_SINGLE_MEAS_);
    }
    /* Process the current measurement */
    mag_cnts_[0] =   static_cast<int16_t>(buf_[1]) << 8 | buf_[0];
    mag_cnts_[1] =   static_cast<int16_t>(buf_[3]) << 8 | buf_[2];
    mag_cnts_[2] =   static_cast<int16_t>(buf_[5]) << 8 | buf_[4];
    if (buf_[6] & ST2_HOFL_) {
      return false;
    }
    mag_[0] =   static_cast<float>(mag_cnts_[0]) * mag_scale_[0];
    mag_[1] =   static_cast<float>(mag_cnts_[1]) * mag_scale_[1];
    mag_[2] =   static_cast<float>(mag_cnts_[2]) * mag_scale_[2];
    new_mag_data_ = true;
    return true;
  }
  return false;
}
void Ak8963::WriteRegister(const uint8_t reg, const uint8_t data) {
  i2c_->beginTransmission(dev_);
  i2c_->write(reg);
  i2c_->write(data);
  i2c_->endTransmission();
}
bool Ak8963::ReadRegisters(const uint8_t reg, const uint8_t count,
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
