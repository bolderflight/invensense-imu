//
// title:     MPU9250.cpp
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      2015-11-07 
// license: 
//

#include "Arduino.h"
#include "MPU9250.h"
#include <i2c_t3.h>  // I2C library

/* MPU9250 object, input the I2C address */
MPU9250::MPU9250(int address){
  _address = address; // I2C address
}

/* starts the I2C communication */
void MPU9250::begin(){
  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400);  // starting the I2C
  writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G); // setting the accel range
  //setGyroRange(gyroRange); // setting the gyro range
}

uint8_t MPU9250::readRegister(uint8_t registerAddress){
  uint8_t avail;

  // write to register to be read
  Wire.beginTransmission(_address);
  Wire.write(registerAddress);
  Wire.endTransmission();

  // request 1 byte back
  Wire.requestFrom(_address,1);

  // check to see how many bytes are available
  avail = Wire.available();
  if(avail == 1){
    return Wire.read();
  }
  else{
    return 0;
  }
}

void MPU9250::writeRegister(uint8_t subAddress, uint8_t data){
  Wire.beginTransmission(_address);
  Wire.write(subAddress);
  Wire.write(data);
  Wire.endTransmission();
}

uint16_t MPU9250::readRegisters(uint8_t msbAddress, uint8_t lsbAddress){

  // write to register to be read
  Wire.beginTransmission(_address);
  Wire.write(msbAddress);
  Wire.endTransmission(false);
  Wire.write(lsbAddress);

  // request 1 byte back
  Wire.requestFrom(_address,1,false);
  uint8_t msb = Wire.read();
  Wire.requestFrom(_address,1);
  uint8_t lsb = Wire.read();

  return (((uint16_t)msb) << 8) | lsb;
}

void MPU9250::getAccel(double* ax, double* ay, double* az){

  *ax = ((int16_t) readRegisters(ACCEL_XOUT_H, ACCEL_XOUT_L)) * G * 4.0/32767.5;
  *ay = ((int16_t) readRegisters(ACCEL_YOUT_H, ACCEL_YOUT_L)) * G * 4.0/32767.5;
  *az = ((int16_t) readRegisters(ACCEL_ZOUT_H, ACCEL_ZOUT_L)) * G * 4.0/32767.5;
}

