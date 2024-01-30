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

#include "mpu9250.h"
#include "ak8963.h"

/* Mpu9250 object */
bfs::Mpu9250 imu(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
/* Mag */
bfs::Ak8963 mag(&Wire);

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Start the I2C bus */
  Wire.begin();
  Wire.setClock(400000);
  /* Initialize and configure IMU */
  if (!imu.Begin(bfs::Mpu9250::MAG_PASSTHROUGH)) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Initialize the mag */
  if (!mag.Begin()) {
    Serial.println("Error initializing communication with mag");
    while(1) {}
  }
  /* Set mag measurement rate */
  if (!mag.ConfigMeasRate(bfs::Ak8963::MEAS_RATE_100HZ)) {
    Serial.println("Error configuring mag measurement rate");
    while(1) {}
  }
}

void loop() {
  /* Check if data read */
  if (mag.Read()) {
    Serial.print(mag.mag_x_ut());
    Serial.print("\t");
    Serial.print(mag.mag_y_ut());
    Serial.print("\t");
    Serial.print(mag.mag_z_ut());
    Serial.print("\n");
  }
}