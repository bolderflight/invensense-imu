/*
Basic_SPI.ino
Brian R Taylor
brian.taylor@bolderflight.com
2016-10-10 

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

// an MPU9250 object with the MPU-9250 sensor on Teensy Chip Select pin 10
MPU9250 IMU(10);

float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
int beginStatus;

void setup() {
  // serial to display data
  Serial.begin(115200);

  // start communication with IMU and 
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS
  beginStatus = IMU.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
}

void loop() {
  if(beginStatus < 0) {
    delay(1000);
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    delay(10000);
  }
  else{
    /* get the individual data sources */
    /* This approach is only recommended if you only
     *  would like the specified data source (i.e. only
     *  want accel data) since multiple data sources
     *  would have a time skew between them.
     */
    // get the accelerometer data (m/s/s)
    IMU.getAccel(&ax, &ay, &az);
  
    // get the gyro data (rad/s)
    IMU.getGyro(&gx, &gy, &gz);
  
    // get the magnetometer data (uT)
    IMU.getMag(&hx, &hy, &hz);
  
    // get the temperature data (C)
    IMU.getTemp(&t);
  
    // print the data
    printData();
  
    // delay a frame
    delay(50);
  
    /* get multiple data sources */
    /* In this approach we get data from multiple data
     *  sources (i.e. both gyro and accel). This is 
     *  the recommended approach since there is no time
     *  skew between sources - they are all synced.
     *  Demonstrated are:
     *  1. getMotion6: accel + gyro
     *  2. getMotion7: accel + gyro + temp
     *  3. getMotion9: accel + gyro + mag
     *  4. getMotion10: accel + gyro + mag + temp
     */
  
     /* getMotion6 */
    // get both the accel (m/s/s) and gyro (rad/s) data
    IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
    // get the magnetometer data (uT)
    IMU.getMag(&hx, &hy, &hz);
  
    // get the temperature data (C)
    IMU.getTemp(&t);
  
    // print the data
    printData();
  
    // delay a frame
    delay(50);
  
    /* getMotion7 */
    // get the accel (m/s/s), gyro (rad/s), and temperature (C) data
    IMU.getMotion7(&ax, &ay, &az, &gx, &gy, &gz, &t);
    
    // get the magnetometer data (uT)
    IMU.getMag(&hx, &hy, &hz);
  
    // print the data
    printData();
  
    // delay a frame
    delay(50);
  
    /* getMotion9 */
    // get the accel (m/s/s), gyro (rad/s), and magnetometer (uT) data
    IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
  
    // get the temperature data (C)
    IMU.getTemp(&t);
  
    // print the data
    printData();
  
    // delay a frame
    delay(50);
  
    // get the accel (m/s/s), gyro (rad/s), and magnetometer (uT), and temperature (C) data
    IMU.getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
  
    // print the data
    printData();
  
    // delay a frame
    delay(50);
  }
}

void printData(){

  // print the data
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
  Serial.print(gz,6);
  Serial.print("\t");

  Serial.print(hx,6);
  Serial.print("\t");
  Serial.print(hy,6);
  Serial.print("\t");
  Serial.print(hz,6);
  Serial.print("\t");

  Serial.println(t,6);
}

