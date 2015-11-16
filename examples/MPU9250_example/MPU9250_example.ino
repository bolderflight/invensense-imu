//
// title:     MPU9250_example.ino
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      2015-11-16 
// license: 
//

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
}
