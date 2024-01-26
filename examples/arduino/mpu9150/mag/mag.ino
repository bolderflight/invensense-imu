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

#include "mpu9150.h"
#include "ak8975.h"

/* Mpu9150 object */
bfs::Mpu9150 imu(&Wire, bfs::Mpu9150::I2C_ADDR_PRIM);

bfs::Ak8975 mag(&Wire);

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  if (!mag.Begin()) {
    Serial.println("Error initializing communication with Mag");
    while(1) {}
  }
  /* Set the sample rate divider */
  // if (!imu.ConfigSrd(19)) {
  //   Serial.println("Error configuring SRD");
  //   while(1) {}
  // }
  Serial.println("INIT COMPLETE");
}

void loop() {
  /* Check if data read */
  // if (imu.Read()) {
  //   Serial.print(imu.new_imu_data());
  //   Serial.print("\t");
  //   Serial.print(imu.accel_x_mps2());
  //   Serial.print("\t");
  //   Serial.print(imu.accel_y_mps2());
  //   Serial.print("\t");
  //   Serial.print(imu.accel_z_mps2());
  //   Serial.print("\t");
  //   Serial.print(imu.gyro_x_radps());
  //   Serial.print("\t");
  //   Serial.print(imu.gyro_y_radps());
  //   Serial.print("\t");
  //   Serial.print(imu.gyro_z_radps());
  //   Serial.print("\t");
  //   Serial.print(imu.die_temp_c());
  //   Serial.print("\t");
  if (mag.Read()) {
    Serial.print(mag.mag_x_ut());
    Serial.print("\t");
    Serial.print(mag.mag_y_ut());
    Serial.print("\t");
    Serial.print(mag.mag_z_ut());
    Serial.println();
  }
}
