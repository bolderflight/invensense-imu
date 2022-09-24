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

#ifndef INVENSENSE_IMU_SRC_BFS_MPU9250_H_  // NOLINT
#define INVENSENSE_IMU_SRC_BFS_MPU9250_H_

#include "mpu9250.h"  // NOLINT
#include "types.h"  // NOLINT
#include "statistics.h"  // NOLINT
#include "eigen.h"  // NOLINT

namespace bfs {

class BfsMpu9250 : public Mpu9250 {
 public:
  enum SampleRate : uint8_t {
    SAMPLE_RATE_1000HZ = 0,
    SAMPLE_RATE_500HZ = 1,
    SAMPLE_RATE_200HZ = 4,
    SAMPLE_RATE_100HZ = 9,
    SAMPLE_RATE_50HZ = 19
  };
  struct Config {
    SampleRate sample_rate = SAMPLE_RATE_100HZ;
    int32_t init_time_ms = 5000;
    Mpu9250::AccelRange accel_range_g = Mpu9250::ACCEL_RANGE_16G;
    Mpu9250::GyroRange gyro_range_dps = Mpu9250::GYRO_RANGE_2000DPS;
    Mpu9250::DlpfBandwidth dlpf_hz = Mpu9250::DLPF_BANDWIDTH_20HZ;
    float accel_bias_mps2[3] = {0, 0, 0};
    float mag_bias_ut[3] = {0, 0, 0};
    float accel_scale[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float mag_scale[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    float rotation[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  };
  BfsMpu9250(TwoWire *i2c, const Mpu9250::I2cAddr addr) : imu_(i2c, addr) {}
  BfsMpu9250(SPIClass *spi, const uint8_t cs) : imu_(spi, cs) {}
  bool Init(const Config &config);
  bool Calibrate();
  void ResetCal();
  bool Read();
  inline ImuData imu_data() const {return data_;}
  inline MagData mag_data() const {return mag_data_;}

 private:
  bool latch_ = false;
  Mpu9250 imu_;
  Config config_;
  ImuData data_;
  MagData mag_data_;
  elapsedMillis time_ms_;
  Eigen::Vector3f accel_bias_mps2_, gyro_bias_radps_, mag_bias_ut_;
  Eigen::Vector3f accel_mps2_, gyro_radps_, mag_ut_;
  Eigen::Matrix3f accel_scale_, mag_scale_;
  Eigen::Matrix3f rotation_;
  RunningStats<float> gx_, gy_, gz_;
};

}  // namespace bfs

#endif  // INVENSENSE_IMU_SRC_BFS_MPU6500_H_ NOLINT
