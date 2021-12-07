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

#include "mpu9250.h"

/* Mpu9250 object, SPI bus, CS on pin 10 */
bfs::Mpu9250 imu(&SPI, 10);
/* Sample rate divider driven by IMU data ready interrupt at 1000 Hz */
volatile int srd = 0;
/* Arrays for copying data */
int8_t fifo_len;
float ax_mps2[bfs::Mpu9250::FIFO_MAX_SIZE()];
float ay_mps2[bfs::Mpu9250::FIFO_MAX_SIZE()];
float az_mps2[bfs::Mpu9250::FIFO_MAX_SIZE()];
float gx_radps[bfs::Mpu9250::FIFO_MAX_SIZE()];
float gy_radps[bfs::Mpu9250::FIFO_MAX_SIZE()];
float gz_radps[bfs::Mpu9250::FIFO_MAX_SIZE()];

/* Data acquisition ISR */
void imu_isr() {
  srd++;
  /* Collect FIFO data at 50 Hz */
  if (srd > 19) {
    srd = 0;
    /* Read the FIFO buffer */
    fifo_len = imu.ReadFifo();
    imu.fifo_accel_x_mps2(ax_mps2, bfs::Mpu9250::FIFO_MAX_SIZE());
    imu.fifo_accel_y_mps2(ay_mps2, bfs::Mpu9250::FIFO_MAX_SIZE());
    imu.fifo_accel_z_mps2(az_mps2, bfs::Mpu9250::FIFO_MAX_SIZE());
    imu.fifo_gyro_x_radps(gx_radps, bfs::Mpu9250::FIFO_MAX_SIZE());
    imu.fifo_gyro_y_radps(gy_radps, bfs::Mpu9250::FIFO_MAX_SIZE());
    imu.fifo_gyro_z_radps(gz_radps, bfs::Mpu9250::FIFO_MAX_SIZE());
    /*
    * Print the FIFO length (should be 20) and the last FIFO sample for all
    * channels as an example. This FIFO buffer could be used to compute delta
    * theta and delta velocity using 1000 Hz data, while having the
    * microcontroller sample the IMU at a much lower rate.
    */
    Serial.print(fifo_len);
    Serial.print("\t");
    Serial.print(ax_mps2[fifo_len - 1]);
    Serial.print("\t");
    Serial.print(ay_mps2[fifo_len - 1]);
    Serial.print("\t");
    Serial.print(az_mps2[fifo_len - 1]);
    Serial.print("\t");
    Serial.print(gx_radps[fifo_len - 1]);
    Serial.print("\t");
    Serial.print(gy_radps[fifo_len - 1]);
    Serial.print("\t");
    Serial.print(gz_radps[fifo_len - 1]);
    Serial.print("\n");
  }
}

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Start the SPI bus */
  SPI.begin();
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Enabled data ready interrupt */
  if (!imu.EnableDrdyInt()) {
    Serial.println("Error enabling data ready interrupt");
    while(1) {}
  }
  /* Enable FIFO */
  if (!imu.EnableFifo()) {
    Serial.println("Error enabling FIFO");
    while(1) {}
  }
  /* Attach data ready interrupt to pin 9 */
  attachInterrupt(9, imu_isr, RISING);
}

void loop() {}
