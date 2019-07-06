/*
Basic_I2C.ino
Brian R Taylor
brian.taylor@bolderflight.com

Copyright (c) 2017 Bolder Flight Systems

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

#include "Arduino.h"
#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 2 with address 0x68
//MPU9250 IMU(Wire,0x68);
MPU9250 IMU(Wire2,0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while(!Serial) {}

  Serial.println("Serial COM Opened");

  Serial.println("Initializing I2C");
  // When if using i2c_t3 refer to the i2c_t3 library for configuration info
  // https://github.com/nox771/i2c_t3
  // Permalink to info on i2c pin selection
  // https://github.com/nox771/i2c_t3/blob/1a482d9f85bfebcd526e68fdea1c99cf83d7c85b/i2c_t3.h#L222
  // If you wish to modify default i2c settings you will need to call Wire.begin here seperately
  // with the additional arguments that you wish to pass in.
  // Setup for Master mode, pins 18/19, external pullups, 400kHz, 200ms default timeout
  //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
  //Wire2.begin(I2C_MASTER, 0x00, I2C_PINS_3_4, I2C_PULLUP_EXT, 400000);
  //Wire2.setDefaultTimeout(200000); // 200ms
  //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, 400000);
  Wire.setDefaultTimeout(200000); // 200ms

   Serial.println("Initializing IMU");
  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  Serial.println("Device Initializations Complete");
}

void loop() {
  // read the sensor
  IMU.readSensor();
  // display the data
  Serial.print(IMU.getAccelX_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelY_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getAccelZ_mss(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroX_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroY_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getGyroZ_rads(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagX_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagY_uT(),6);
  Serial.print("\t");
  Serial.print(IMU.getMagZ_uT(),6);
  Serial.print("\t");
  Serial.println(IMU.getTemperature_C(),6);
  delay(100);
}
