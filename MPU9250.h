//
// title:     MPU9250.h
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      2015-11-07 
// license: 
//

#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"

#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40

#define ACCEL_CONFIG		0x1C
#define ACCEL_FS_SEL_2G		0x00
#define	ACCEL_FS_SEL_4G		0x08
#define ACCEL_FS_SEL_8G 	0x10
#define ACCEL_FS_SEL_16G	0x18

#define G			9.807						



class MPU9250{
  public:
    MPU9250(int address);
    void begin();
    void getAccel(double* ax, double* ay, double* az);
  private:
    int _address;
    void writeRegister(uint8_t subAddress, uint8_t data);
    uint8_t readRegister(uint8_t registerAddress);
    uint16_t readRegisters(uint8_t msbAddress, uint8_t lsbAddress);
};

#endif
