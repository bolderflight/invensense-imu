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

#ifndef INVENSENSE_IMU_SRC_INVENSENSE_IMU_H_  // NOLINT
#define INVENSENSE_IMU_SRC_INVENSENSE_IMU_H_

#if defined(ARDUINO)
#include <Arduino.h>
#include "Wire.h"
#include "SPI.h"
#else
#include <cstddef>
#include <cstdint>
#include "core/core.h"
#endif

namespace bfs {

class InvensenseImu {
 public:
  InvensenseImu() {}
  InvensenseImu(TwoWire *i2c, const uint8_t addr) : i2c_(i2c),
                                                    dev_(addr),
                                                    iface_(I2C) {}
  InvensenseImu(SPIClass *spi, const uint8_t cs) : spi_(spi),
                                                   dev_(cs),
                                                   iface_(SPI) {}
  void Config(TwoWire *i2c, const uint8_t addr);
  void Config(SPIClass *spi, const uint8_t cs);
  void Begin();
  bool ReadRegisters(const uint8_t reg, const uint8_t count,
                     const int32_t spi_clock, uint8_t * const data);
  bool WriteRegister(const uint8_t reg, const uint8_t data);
  bool WriteRegister(const uint8_t reg, const uint8_t data,
                     const int32_t spi_clock);
  bool ReadRegisters(const uint8_t reg, const uint8_t count,
                     uint8_t * const data);

 private:
  /* Communications interface */
  enum Interface : int8_t {
    SPI,
    I2C
  };
  TwoWire *i2c_;
  SPIClass *spi_;
  uint8_t dev_;
  Interface iface_;
  uint8_t bytes_rx_;
  /* SPI flag to indicate a read operation */
  static constexpr uint8_t SPI_READ_ = 0x80;
};

}  // namespace bfs

#endif  // INVENSENSE_IMU_SRC_INVENSENSE_IMU_H_ NOLINT
