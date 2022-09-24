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

#include "bfs-mpu9250.h"

bfs::BfsMpu9250 imu(&SPI, 10);

int main() {
  Serial.begin(115200);
  while (!Serial) {}
  SPI.begin();
  bfs::BfsMpu9250::Config config = {
    .sample_rate = bfs::BfsMpu9250::SAMPLE_RATE_1000HZ,
    .init_time_ms = 5000,
    .accel_range_g = bfs::Mpu9250::ACCEL_RANGE_16G,
    .gyro_range_dps = bfs::Mpu9250::GYRO_RANGE_2000DPS,
    .dlpf_hz = bfs::Mpu9250::DLPF_BANDWIDTH_20HZ,
    .accel_bias_mps2 = {0, 0, 0},
    .accel_scale = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}},
    .rotation = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}
  };
  bfs::ImuData imu_data;
  bfs::MagData mag_data;
  if (!imu.Init(config)) {
    Serial.println("Failed to init sensor");
  }
  while (!imu.Calibrate()) {}
  while (1) {
    if (imu.Read()) {
      imu_data = imu.imu_data();
      mag_data = imu.mag_data();
      Serial.print(imu_data.new_data);
      Serial.print("\t");
      Serial.print(imu_data.accel_mps2[0]);
      Serial.print("\t");
      Serial.print(imu_data.accel_mps2[1]);
      Serial.print("\t");
      Serial.print(imu_data.accel_mps2[2]);
      Serial.print("\t");
      Serial.print(imu_data.gyro_radps[0]);
      Serial.print("\t");
      Serial.print(imu_data.gyro_radps[1]);
      Serial.print("\t");
      Serial.print(imu_data.gyro_radps[2]);
      Serial.print("\t");
      Serial.print(imu_data.die_temp_c);
      Serial.print("\t");
      Serial.print(mag_data.new_data);
      Serial.print("\t");
      Serial.print(mag_data.mag_ut[0]);
      Serial.print("\t");
      Serial.print(mag_data.mag_ut[1]);
      Serial.print("\t");
      Serial.print(mag_data.mag_ut[2]);
      Serial.print("\t");
      Serial.print(mag_data.die_temp_c);
      Serial.print("\n");
    }
  }
}
