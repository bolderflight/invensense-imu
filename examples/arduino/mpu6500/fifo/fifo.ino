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

#include "mpu6500.h"

/* Mpu6500 object, SPI bus, CS on pin 10 */
bfs::Mpu6500 imu(&SPI, 10);

void setup() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  delay(1000);
  Serial.println("Starting example");
  /* Start the SPI bus */
  SPI.begin();
  /* Initialize and configure IMU */
  if (!imu.Begin()) {
    Serial.println("Error initializing communication with IMU");
    while(1) {}
  }
  /* Set the sample rate divider */
  if (!imu.ConfigSrd(19)) {
    Serial.println("Error configuring SRD");
    while(1) {}
  }
  /* Enable FIFO */
  if (!imu.EnableFifo()) {
    Serial.println("Error enabling FIFO");
    while(1) {}
  }
  /* Delay and read FIFO */
  uint8_t buf[1024];
  float gx[128], gy[128], gz[128], ax[128], ay[128], az[128];
  delay(100);
  int16_t bytes_read = imu.ReadFifo(buf, sizeof(buf));  // should be 5 samples of the IMU
  int16_t num_records = imu.ProcessFifoData(buf, bytes_read, gx, gy, gz, ax, ay, az);
  for (size_t i = 0; i < num_records; i++) {
    Serial.print(ax[i]);
    Serial.print("\t");
    Serial.print(ay[i]);
    Serial.print("\t");
    Serial.print(az[i]);
    Serial.print("\t");
    Serial.print(gx[i]);
    Serial.print("\t");
    Serial.print(gy[i]);
    Serial.print("\t");
    Serial.print(gz[i]);
    Serial.print("\n");
  }
}

void loop() {}
