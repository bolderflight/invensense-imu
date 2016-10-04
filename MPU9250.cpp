/*
MPU9250.cpp
Brian R Taylor
brian.taylor@bolderflight.com
2016-06-08

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

#include "Arduino.h"
#include "MPU9250.h"
#include <i2c_t3.h>  // I2C library

/* MPU9250 object, input the I2C address */
MPU9250::MPU9250(int address){
  _address = address; // I2C address
}

/* starts the I2C communication */
int MPU9250::begin(String accelRange, String gyroRange){
	uint8_t accelSetting, gyroSetting;

	Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_400); // starting the I2C

	// reset the device
	writeRegister(PWR_MGMNT_1,PWR_RESET);

	/* setup the accel and gyro ranges */
	if(accelRange.equals("2G")){
  		accelSetting = ACCEL_FS_SEL_2G; // setting the accel range to 2G
    	_accelScale = G * 2.0/32767.5; // setting the accel scale to 2G
	}

	if(accelRange.equals("4G")){
  		accelSetting = ACCEL_FS_SEL_4G; // setting the accel range to 4G
    	_accelScale = G * 4.0/32767.5; // setting the accel scale to 4G
	}

	if(accelRange.equals("8G")){
  		accelSetting = ACCEL_FS_SEL_8G; // setting the accel range to 8G
    	_accelScale = G * 8.0/32767.5; // setting the accel scale to 8G
	}

	if(accelRange.equals("16G")){
  		accelSetting = ACCEL_FS_SEL_16G; // setting the accel range to 16G
    	_accelScale = G * 16.0/32767.5; // setting the accel scale to 16G
	}

	if(gyroRange.equals("250DPS")){
  		gyroSetting = GYRO_FS_SEL_250DPS; // setting the gyro range to 250DPS
    	_gyroScale = 250.0/32767.5; // setting the gyro scale to 250DPS
	}

	if(gyroRange.equals("500DPS")){
  		gyroSetting = GYRO_FS_SEL_500DPS; // setting the gyro range to 500DPS
    	_gyroScale = 500.0/32767.5; // setting the gyro scale to 500DPS
	}

	if(gyroRange.equals("1000DPS")){
  		gyroSetting = GYRO_FS_SEL_1000DPS; // setting the gyro range to 1000DPS
    	_gyroScale = 1000.0/32767.5; // setting the gyro scale to 1000DPS
	}

	if(gyroRange.equals("2000DPS")){
  		gyroSetting = GYRO_FS_SEL_2000DPS; // setting the gyro range to 2000DPS
    	_gyroScale = 2000.0/32767.5; // setting the gyro scale to 2000DPS
	}

	// wait for oscillators to stabilize
	delayMicroseconds(500);

	// select clock source to gyro
	if( !writeRegister(PWR_MGMNT_1,CLOCK_SEL_PLL) ){
		return -1;
	}

	// check the WHO AM I byte, expected value is 0x71
	if( whoAmI() != 0x71 ){
  		return -1;
	}

	// enable accelerometer and gyro
	if( !writeRegister(PWR_MGMNT_2,SEN_ENABLE) ){
		return -1;
	}

	// write the accelerometer settings
	if( !writeRegister(ACCEL_CONFIG,accelSetting) ){
		return -1;
	}

	// write the gyro settings
	if( !writeRegister(GYRO_CONFIG,gyroSetting) ){
		return -1;
	}

  return 0;
}

/* sets the DLPF and interrupt settings */
void MPU9250::setFilt(String bandwidth, uint8_t frequency){
  uint16_t ISR = 1000; // with all of the DLPF settings below, the frequency will be 1 kHz
  uint8_t SRD; // sample rate divider

  if(bandwidth.equals("184HZ")){
    writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_184); // setting accel bandwidth to 184Hz
	  writeRegister(CONFIG,GYRO_DLPF_184); // setting gyro bandwidth to 184Hz
  }

  if(bandwidth.equals("92HZ")){
    writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_92); // setting accel bandwidth to 92Hz
	  writeRegister(CONFIG,GYRO_DLPF_92); // setting gyro bandwidth to 92Hz
  }

  if(bandwidth.equals("41HZ")){
    writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_41); // setting accel bandwidth to 41Hz
	  writeRegister(CONFIG,GYRO_DLPF_41); // setting gyro bandwidth to 41Hz
  }

  if(bandwidth.equals("20HZ")){
    writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_20); // setting accel bandwidth to 20Hz
	  writeRegister(CONFIG,GYRO_DLPF_20); // setting gyro bandwidth to 20Hz
  }

  if(bandwidth.equals("10HZ")){
    writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_10); // setting accel bandwidth to 10Hz
	  writeRegister(CONFIG,GYRO_DLPF_10); // setting gyro bandwidth to 10Hz
  }

  if(bandwidth.equals("5HZ")){
    writeRegister(ACCEL_CONFIG2,ACCEL_DLPF_5); // setting accel bandwidth to 5Hz
	  writeRegister(CONFIG,GYRO_DLPF_5); // setting gyro bandwidth to 5Hz
  }

  SRD = ISR / frequency - 1; // determining the correct sample rate divider to get the desired frequency

  writeRegister(SMPDIV,SRD); // setting the sample rate divider

  writeRegister(INT_PIN_CFG,0x00);	// setup interrupt, 50 us pulse
  writeRegister(INT_ENABLE,0x01);	// set to data ready
}

/* writes a register to MPU9250 given a register address and data */
bool MPU9250::writeRegister(uint8_t subAddress, uint8_t data){
	uint8_t buff[1];

	/* write data to device */
  	Wire.beginTransmission(_address); // open the device
  	Wire.write(subAddress); // write the register address
  	Wire.write(data); // write the data
  	Wire.endTransmission();

  	/* read back the register */
  	readRegisters(subAddress,sizeof(buff),&buff[0]);

  	/* check the read back register against the written register */
  	if(buff[0] == data) {
  		return true;
  	}
  	else{
  		return false;
  	}
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

/* get accelerometer data given pointers to store the three values, return data as counts */
void MPU9250::getAccelCounts(uint16_t* ax, uint16_t* ay, uint16_t* az){
  uint8_t buff[6];

  readRegisters(ACCEL_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

  *ax = (((uint16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
  *ay = (((uint16_t)buff[2]) << 8) | buff[3];
  *az = (((uint16_t)buff[4]) << 8) | buff[5];
}

/* get accelerometer data given pointers to store the three values */
void MPU9250::getAccel(double* ax, double* ay, double* az){
  uint16_t accel[3];

  getAccelCounts(&accel[0], &accel[1], &accel[2]);

  *ax = ((int16_t) accel[0]) * _accelScale; // typecast and scale to values
  *ay = ((int16_t) accel[1]) * _accelScale;
  *az = ((int16_t) accel[2]) * _accelScale;
}

/* get gyro data given pointers to store the three values, return data as counts */
void MPU9250::getGyroCounts(uint16_t* gx, uint16_t* gy, uint16_t* gz){
  uint8_t buff[6];

  readRegisters(GYRO_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

  *gx = (((uint16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit values
  *gy = (((uint16_t)buff[2]) << 8) | buff[3];
  *gz = (((uint16_t)buff[4]) << 8) | buff[5];
}

/* get gyro data given pointers to store the three values */
void MPU9250::getGyro(double* gx, double* gy, double* gz){
  uint16_t gyro[3];

  getGyroCounts(&gyro[0], &gyro[1], &gyro[2]);

  *gx = ((int16_t) gyro[0]) * _gyroScale; // typecast and scale to values
  *gy = ((int16_t) gyro[1]) * _gyroScale;
  *gz = ((int16_t) gyro[2]) * _gyroScale;
}

/* get temperature data given pointers to store the three values, return data as counts */
void MPU9250::getTempCounts(uint16_t* t){
	uint8_t buff[2];

	readRegisters(TEMP_OUT, sizeof(buff), &buff[0]); // grab the data from the MPU9250

	*t = (((uint16_t)buff[0]) << 8) | buff[1];  // combine into 16 bit value and return
}

/* get temperature data given pointers to store the three values */
void MPU9250::getTemp(double* t){
	uint16_t tempCount;
	float sens = 333.87f;
	float offset = 21.0f;

	getTempCounts(&tempCount);

	*t = (( ((int16_t) tempCount) - offset )/sens) + 21.0f;
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

/* get accelerometer and gyro data given pointers to store values */
void MPU9250::getMotion6(double* ax, double* ay, double* az, double* gx, double* gy, double* gz){
  uint16_t accel[3];
  uint16_t gyro[3];

  getMotion6Counts(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2]);

  *ax = ((int16_t) accel[0]) * _accelScale; // typecast and scale to values
  *ay = ((int16_t) accel[1]) * _accelScale;
  *az = ((int16_t) accel[2]) * _accelScale;

  *gx = ((int16_t) gyro[0]) * _gyroScale;
  *gy = ((int16_t) gyro[1]) * _gyroScale;
  *gz = ((int16_t) gyro[2]) * _gyroScale;
}

/* gets the MPU-9250 WHO_AM_I register value, expected to be 0x71 */
uint8_t MPU9250::whoAmI(){
	uint8_t buff[1];

	// read the WHO AM I register
	readRegisters(WHO_AM_I,sizeof(buff),&buff[0]);

	// return the register value
	return buff[0];
}
