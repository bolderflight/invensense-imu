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

/* Mpu9250 object using SPI */
bfs::Mpu9250 mpu9250(&SPI, 10);
/* Data acquisition ISR */
void imu_isr() {
  /* Check if data read */
  if (mpu9250.Read()) {
    /* Print data */
    Serial.print(mpu9250.accel_x_mps2());
    Serial.print("\t");
    Serial.print(mpu9250.accel_y_mps2());
    Serial.print("\t");
    Serial.print(mpu9250.accel_z_mps2());
    Serial.print("\t");
    Serial.print(mpu9250.gyro_x_radps());
    Serial.print("\t");
    Serial.print(mpu9250.gyro_y_radps());
    Serial.print("\t");
    Serial.print(mpu9250.gyro_z_radps());
    Serial.print("\t");
    Serial.print(mpu9250.mag_x_ut());
    Serial.print("\t");
    Serial.print(mpu9250.mag_y_ut());
    Serial.print("\t");
    Serial.print(mpu9250.mag_z_ut());
    Serial.print("\t");
    Serial.print(mpu9250.die_temperature_c());
    Serial.print("\n");
  }
}

int main() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Start communicating with MPU-9250 */
  bool status = mpu9250.Begin();
  if (!status) {
    Serial.println("ERROR: unable to communicate with MPU-9250");
    while(1) {}
  }
  /* Set sample rate divider for 50 Hz */
  status = mpu9250.ConfigSrd(19);
  if (!status) {
    Serial.println("ERROR: unable to setup sample rate divider");
    while(1) {}
  }
  /* Enable the data ready interrupt */
  status = mpu9250.EnableDrdyInt();
  if (!status) {
    Serial.println("ERROR: unable to set data ready interrupt");
    while(1) {}
  }
  /* Setup callback for data ready interrupt */
  mpu9250.DrdyCallback(3, imu_isr);
  while (1) {}
}

