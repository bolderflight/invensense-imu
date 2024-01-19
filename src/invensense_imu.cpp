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

#include "invensense_imu.h"  // NOLINT

namespace bfs {

void InvensenseImu::Config(TwoWire *i2c, const uint8_t addr) {
  i2c_ = i2c;
  dev_ = addr;
  iface_ = I2C;
}

void InvensenseImu::Config(SPIClass *spi, const uint8_t cs) {
  spi_ = spi;
  dev_ = cs;
  iface_ = SPI;
}

void InvensenseImu::Begin() {
  if (iface_ == SPI) {
    pinMode(dev_, OUTPUT);
    /* Toggle CS pin to lock in SPI mode */
    digitalWrite(dev_, LOW);
    delay(1);
    digitalWrite(dev_, HIGH);
    delay(1);
  }
}

bool InvensenseImu::WriteRegister(const uint8_t reg, const uint8_t data,
                                  const int32_t spi_clock) {
  uint8_t ret_val;
  if (iface_ == I2C) {
    i2c_->beginTransmission(dev_);
    i2c_->write(reg);
    i2c_->write(data);
    i2c_->endTransmission();
  } else {
    spi_->beginTransaction(SPISettings(spi_clock, MSBFIRST, SPI_MODE3));
    #if defined(TEENSYDUINO)
    digitalWriteFast(dev_, LOW);
    #else
    digitalWrite(dev_, LOW);
    #endif
    #if defined(__IMXRT1062__)
      delayNanoseconds(125);
    #endif
    spi_->transfer(reg);
    spi_->transfer(data);
    #if defined(TEENSYDUINO)
    digitalWriteFast(dev_, HIGH);
    #else
    digitalWrite(dev_, HIGH);
    #endif
    #if defined(__IMXRT1062__)
      delayNanoseconds(125);
    #endif
    spi_->endTransaction();
  }
  delay(10);
  ReadRegisters(reg, sizeof(ret_val), spi_clock, &ret_val);
  if (data == ret_val) {
    return true;
  } else {
    return false;
  }
}

bool InvensenseImu::ReadRegisters(const uint8_t reg, const uint8_t count,
                                  const int32_t spi_clock,
                                  uint8_t * const data) {
  if (!data) {return false;}
  if (iface_ == I2C) {
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
  } else {
    spi_->beginTransaction(SPISettings(spi_clock, MSBFIRST, SPI_MODE3));
    #if defined(TEENSYDUINO)
    digitalWriteFast(dev_, LOW);
    #else
    digitalWrite(dev_, LOW);
    #endif
    #if defined(__IMXRT1062__)
      delayNanoseconds(125);
    #endif
    spi_->transfer(reg | SPI_READ_);
    spi_->transfer(data, count);
    #if defined(TEENSYDUINO)
    digitalWriteFast(dev_, HIGH);
    #else
    digitalWrite(dev_, HIGH);
    #endif
    #if defined(__IMXRT1062__)
      delayNanoseconds(125);
    #endif
    spi_->endTransaction();
    return true;
  }
}

bool InvensenseImu::WriteRegister(const uint8_t reg, const uint8_t data) {
  if (iface_ == I2C) {
    return WriteRegister(reg, data, 0);
  } else {
    return false;
  }
}

bool InvensenseImu::ReadRegisters(const uint8_t reg, const uint8_t count,
                                  uint8_t * const data) {
  if (iface_ == I2C) {
    return ReadRegisters(reg, count, 0, data);
  } else {
    return false;
  }
}

}  // namespace bfs
