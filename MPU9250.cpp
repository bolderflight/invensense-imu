//
// title:     MPU9250.cpp
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      2015-11-19 
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
void MPU9250::begin(String accelRange, String gyroRange){
  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400); // starting the I2C

  writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL); // select clock source to gyro

  if(accelRange.equals("2G")){
    writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_2G); // setting the accel range to 2G
    _accelScale = G * 2.0/32767.5; // setting the accel scale to 2G
  }

  if(accelRange.equals("4G")){
    writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_4G); // setting the accel range to 4G
    _accelScale = G * 4.0/32767.5; // setting the accel scale to 4G
  }

  if(accelRange.equals("8G")){
    writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_8G); // setting the accel range to 8G
    _accelScale = G * 8.0/32767.5; // setting the accel scale to 8G
  }

  if(accelRange.equals("16G")){
    writeRegister(ACCEL_CONFIG,ACCEL_FS_SEL_16G); // setting the accel range to 16G
    _accelScale = G * 16.0/32767.5; // setting the accel scale to 16G
  }

  if(gyroRange.equals("250DPS")){
    writeRegister(GYRO_CONFIG,GYRO_FS_SEL_250DPS); // setting the gyro range to 250DPS
    _gyroScale = 250.0/32767.5; // setting the gyro scale to 250DPS
  }

  if(gyroRange.equals("500DPS")){
    writeRegister(GYRO_CONFIG,GYRO_FS_SEL_500DPS); // setting the gyro range to 500DPS
    _gyroScale = 500.0/32767.5; // setting the gyro scale to 500DPS
  }

  if(gyroRange.equals("1000DPS")){
    writeRegister(GYRO_CONFIG,GYRO_FS_SEL_1000DPS); // setting the gyro range to 1000DPS
    _gyroScale = 1000.0/32767.5; // setting the gyro scale to 1000DPS
  }

  if(gyroRange.equals("2000DPS")){
    writeRegister(GYRO_CONFIG,GYRO_FS_SEL_2000DPS); // setting the gyro range to 2000DPS
    _gyroScale = 2000.0/32767.5; // setting the gyro scale to 2000DPS
  }
}

/* writes a register to MPU9250 given a register address and data */
void MPU9250::writeRegister(uint8_t subAddress, uint8_t data){
  Wire.beginTransmission(_address); // open the device
  Wire.write(subAddress); // write the register address
  Wire.write(data); // write the data
  Wire.endTransmission();
}

/* reads registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void MPU9250::readRegisters(uint8_t subAddress, int count, uint8_t* dest){
  Wire.beginTransmission(_address); // open the device
  Wire.write(subAddress); // specify the starting register address
  Wire.endTransmission(false);

  Wire.requestFrom(_address, count); // specify the number of bytes to receive

  uint8_t i = 0; // read the data into the buffer
  while(Wire.available()){
    dest[i++] = Wire.read();
  }
}

/* get accelerometer data given pointers to store the three values */
void MPU9250::getAccel(double* ax, double* ay, double* az){
  uint8_t buff[6];
  uint16_t accel[3];

  readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

  accel[0] = (((uint16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
  accel[1] = (((uint16_t)buff[2]) << 8) | buff[3];
  accel[2] = (((uint16_t)buff[4]) << 8) | buff[5];

  *ax = ((int16_t) accel[0]) * _accelScale; // typecast and scale to values
  *ay = ((int16_t) accel[1]) * _accelScale;
  *az = ((int16_t) accel[2]) * _accelScale;
}

/* get gyro data given pointers to store the three values */
void MPU9250::getGyro(double* gx, double* gy, double* gz){
  uint8_t buff[6];
  uint16_t gyro[3];

  readRegisters(GYRO_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

  gyro[0] = (((uint16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
  gyro[1] = (((uint16_t)buff[2]) << 8) | buff[3];
  gyro[2] = (((uint16_t)buff[4]) << 8) | buff[5];

  *gx = ((int16_t) gyro[0]) * _gyroScale; // typecast and scale to values
  *gy = ((int16_t) gyro[1]) * _gyroScale;
  *gz = ((int16_t) gyro[2]) * _gyroScale;
}

/* get accelerometer and gyro data given pointers to store values */
void MPU9250::getMotion6(double* ax, double* ay, double* az, double* gx, double* gy, double* gz){
  uint8_t buff[14];
  uint16_t accel[3];
  uint16_t gyro[3];

  readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

  accel[0] = (((uint16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
  accel[1] = (((uint16_t)buff[2]) << 8) | buff[3];
  accel[2] = (((uint16_t)buff[4]) << 8) | buff[5];

  gyro[0] = (((uint16_t)buff[8]) << 8) | buff[9];
  gyro[1] = (((uint16_t)buff[10]) << 8) | buff[11];
  gyro[2] = (((uint16_t)buff[12]) << 8) | buff[13];

  *ax = ((int16_t) accel[0]) * _accelScale; // typecast and scale to values
  *ay = ((int16_t) accel[1]) * _accelScale;
  *az = ((int16_t) accel[2]) * _accelScale;

  *gx = ((int16_t) gyro[0]) * _gyroScale;
  *gy = ((int16_t) gyro[1]) * _gyroScale;
  *gz = ((int16_t) gyro[2]) * _gyroScale;
}

/* get accelerometer and gyro data given pointers to store values, return data as counts */
void MPU9250::getMotion6Counts(uint16_t* ax, uint16_t* ay, uint16_t* az, uint16_t* gx, uint16_t* gy, uint16_t* gz){
  uint8_t buff[14];

  readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

  *ax = (((uint16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
  *ay = (((uint16_t)buff[2]) << 8) | buff[3];
  *az = (((uint16_t)buff[4]) << 8) | buff[5];

  *gx = (((uint16_t)buff[8]) << 8) | buff[9];
  *gy = (((uint16_t)buff[10]) << 8) | buff[11];
  *gz = (((uint16_t)buff[12]) << 8) | buff[13];
}

