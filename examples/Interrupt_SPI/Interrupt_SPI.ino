/*
Interrupt_SPI.ino
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
int beginStatus, setFiltStatus;
volatile int i = 0;

void setup() {
  // serial to display data
  Serial.begin(115200);

  // start communication with IMU and 
  // set the accelerometer and gyro ranges.
  // ACCELEROMETER 2G 4G 8G 16G
  // GYRO 250DPS 500DPS 1000DPS 2000DPS
  beginStatus = IMU.begin(ACCEL_RANGE_4G,GYRO_RANGE_250DPS);
  
  if(beginStatus < 0) {
    delay(1000);
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    while(1){};
  }

  // set up the IMU DLPF, data output rate,
  // and interrupt. DLPF set to 41 Hz, 
  // data output rate set to 100 Hz, and
  // MPU-9250 generated interrupt attached
  // to Teensy pin 2
  setFiltStatus = IMU.setFilt(DLPF_BANDWIDTH_41HZ,9);
  if(setFiltStatus < 0) {
    delay(1000);
    Serial.println("Filter initialization unsuccessful");
    while(1){};
  }
  pinMode(2,INPUT);
  attachInterrupt(2,getIMU,RISING);
}

void loop() {

}

void getIMU(){
  
  // get the accel (m/s/s), gyro (rad/s), and magnetometer (uT), and temperature (C) data
  IMU.getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
  i++;
  // print the results every 10 frames
  if(i > 9){
    printData();
    i = 0;
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

