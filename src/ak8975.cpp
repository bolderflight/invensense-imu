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

#include "ak8975.h"  // NOLINT
#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#else
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include "core/core.h"
#endif

namespace bfs {

void Ak8975::Config(TwoWire *i2c) {
  i2c_ = i2c;
}
bool Ak8975::Begin() {
  delay(100);
  /* Check WHO AM I */
  if (!ReadRegisters(WHOAMI_, 1, buf_)) {
    return false;
  }
  if (buf_[0] != WHOAMI_AK8975_) {
    return false;
  }
  /* Enter fuse ROM mode */
  WriteRegister(CNTL_, FUSE_ROM_);
  delay(100);
  /* Get fuse registers */
  if (!ReadRegisters(ASA_, 3, buf_)) {
    return false;
  }
  mag_scale_[0] = ((static_cast<float>(buf_[0]) - 128.0f)
    / 256.0f + 1.0f) * 1229.0f / 4095.0f;
  mag_scale_[1] = ((static_cast<float>(buf_[1]) - 128.0f)
    / 256.0f + 1.0f) * 1229.0f / 4095.0f;
  mag_scale_[2] = ((static_cast<float>(buf_[2]) - 128.0f)
    / 256.0f + 1.0f) * 1229.0f / 4095.0f;
  /* Set power mode */
  WriteRegister(CNTL_, SINGLE_MEAS_);
  delay(100);
  return true;
}
bool Ak8975::Read() {
    new_mag_data_ = false;
  if (!ReadRegisters(STATUS1_, sizeof(buf_), buf_)) {
    return false;
  }
  /* Check data ready */
  if (buf_[0] & STATUS1_DRDY_) {
    /* Request another measurement */
    WriteRegister(CNTL_, SINGLE_MEAS_);
    /* Process the current measurement */
    mag_cnts_[0] =   static_cast<int16_t>(buf_[2]) << 8 | buf_[1];
    mag_cnts_[1] =   static_cast<int16_t>(buf_[4]) << 8 | buf_[3];
    mag_cnts_[2] =   static_cast<int16_t>(buf_[6]) << 8 | buf_[5];
    if ((buf_[7] & STATUS2_DERR_) || (buf_[7] & STATUS2_HOFL_)) {
      return false;
    }
    new_mag_data_ = true;
    mag_[0] =   static_cast<float>(mag_cnts_[0]) * mag_scale_[0];
    mag_[1] =   static_cast<float>(mag_cnts_[1]) * mag_scale_[1];
    mag_[2] =   static_cast<float>(mag_cnts_[2]) * mag_scale_[2];
    return true;
  }
  return false;
}
void Ak8975::WriteRegister(const uint8_t reg, const uint8_t data) {
  i2c_->beginTransmission(dev_);
  i2c_->write(reg);
  i2c_->write(data);
  i2c_->endTransmission();
}
bool Ak8975::ReadRegisters(const uint8_t reg, const uint8_t count,
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
