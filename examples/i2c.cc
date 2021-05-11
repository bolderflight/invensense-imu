/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2021 Bolder Flight Systems Inc
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

#include "mpu9250/mpu9250.h"

#include "mpu9250/mpu9250.h"

/* Mpu9250 object */
bfs::Mpu9250 imu;

/* IMU data */
bfs::ImuData data;

/* Data acquisition ISR */
void imu_isr() {
  /* Check if data read */
  if (imu.Read(&data)) {
    Serial.print(data.new_imu_data);
    Serial.print("\t");
    Serial.print(data.new_mag_data);
    Serial.print("\t");
    Serial.print(data.imu_healthy);
    Serial.print("\t");
    Serial.print(data.mag_healthy);
    Serial.print("\t");
    Serial.print(data.accel_mps2[0]);
    Serial.print("\t");
    Serial.print(data.accel_mps2[1]);
    Serial.print("\t");
    Serial.print(data.accel_mps2[2]);
    Serial.print("\t");
    Serial.print(data.gyro_radps[0]);
    Serial.print("\t");
    Serial.print(data.gyro_radps[1]);
    Serial.print("\t");
    Serial.print(data.gyro_radps[2]);
    Serial.print("\t");
    Serial.print(data.mag_ut[0]);
    Serial.print("\t");
    Serial.print(data.mag_ut[1]);
    Serial.print("\t");
    Serial.print(data.mag_ut[2]);
    Serial.print("\t");
    Serial.print(data.die_temp_c);
    Serial.print("\n");
  }
}

int main() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Config */
  bfs::ImuConfig config = {
    .dev = 0x68,
    .frame_rate = bfs::FRAME_RATE_50HZ,
    .bus = &Wire,
    .accel_bias_mps2 = {0, 0, 0},
    .mag_bias_ut = {0, 0, 0},
    .accel_scale = {{1, 0, 0},
                    {0, 1, 0},
                    {0, 0, 1}},
    .mag_scale = {{1, 0, 0},
                  {0, 1, 0},
                  {0, 0, 1}},
    .rotation = {{1, 0, 0},
                 {0, 1, 0},
                 {0, 0, 1}}
  };
  /* Init the bus */
  Wire.begin();
  Wire.setClock(400000);
  /* Initialize and configure IMU */
  if (!imu.Init(config)) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Attach data ready interrupt */
  attachInterrupt(27, imu_isr, RISING);
}
