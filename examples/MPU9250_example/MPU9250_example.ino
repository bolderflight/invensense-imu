/*
MPU9250_example.ino
Brian R Taylor
brian.taylor@bolderflight.com
2015-11-19 

Copyright (c) 2016 Bolder Flight Systems

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "MPU9250.h"
#include <i2c_t3.h> // I2C library

// an MPU9250 object with its I2C address 
// of 0x68 (ADDR to GRND)
MPU9250 IMU(0x68);

void setup() {
  // serial to display data
  Serial.begin(9600);

  // start communication with IMU and 
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS
  IMU.begin("4G","250DPS");
}

void loop() {
  double ax, ay, az, gx, gy, gz;
  uint16_t axc, ayc, azc, gxc, gyc, gzc;

  // get the accelerometer data (m/s/s)
  IMU.getAccel(&ax, &ay, &az);
  Serial.print(ax,6);
  Serial.print("\t");
  Serial.print(ay,6);
  Serial.print("\t");
  Serial.print(az,6);
  Serial.print("\t");

  // get the gyro data (deg/s)
  IMU.getGyro(&gx, &gy, &gz);
  Serial.print(gx,6);
  Serial.print("\t");
  Serial.print(gy,6);
  Serial.print("\t");
  Serial.println(gz,6);
  delay(500);

  // get both the accel (m/s/s) and gyro (deg/s) data
  IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  Serial.print(ax,6);
  Serial.print("\t");
  Serial.print(ay,6);
  Serial.print("\t");
  Serial.print(az,6);
  Serial.print("\t");
  Serial.print(gx,6);
  Serial.print("\t");
  Serial.print(gy,6);
  Serial.print("\t");
  Serial.println(gz,6);
  delay(500);

  // get the accelerometer data, in counts
  IMU.getAccelCounts(&axc, &ayc, &azc);
  Serial.print(((int16_t) axc) * 4.0 * 9.807/32767.5, 6);
  Serial.print("\t");
  Serial.print(((int16_t) ayc) * 4.0 * 9.807/32767.5, 6);
  Serial.print("\t");
  Serial.print(((int16_t) azc) * 4.0 * 9.807/32767.5, 6);
  Serial.print("\t");
  
  // get the gyro data, in counts
  IMU.getGyroCounts(&gxc, &gyc, &gzc);
  Serial.print(((int16_t) gxc) * 250.0/32767.5, 6);
  Serial.print("\t");
  Serial.print(((int16_t) gyc) * 250.0/32767.5, 6);
  Serial.print("\t");
  Serial.println(((int16_t) gzc) * 250.0/32767.5, 6);
  delay(500);

  // get both the accel (m/s/s) and gyro data, in counts
  IMU.getMotion6Counts(&axc, &ayc, &azc, &gxc, &gyc, &gzc);
  Serial.print(((int16_t) axc) * 4.0 * 9.807/32767.5, 6);
  Serial.print("\t");
  Serial.print(((int16_t) ayc) * 4.0 * 9.807/32767.5, 6);
  Serial.print("\t");
  Serial.print(((int16_t) azc) * 4.0 * 9.807/32767.5, 6);
  Serial.print("\t");
  Serial.print(((int16_t) gxc) * 250.0/32767.5, 6);
  Serial.print("\t");
  Serial.print(((int16_t) gyc) * 250.0/32767.5, 6);
  Serial.print("\t");
  Serial.println(((int16_t) gzc) * 250.0/32767.5, 6);
  delay(500);
}
