//
// title:     MPU9250.h
// author:    Taylor, Brian R.
// email:     brian.taylor@bolderflight.com
// date:      2015-11-15 
// license: 
//

#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"

#define ACCEL_OUT		0x3B
#define GYRO_OUT		0x43

#define ACCEL_CONFIG		0x1C
#define ACCEL_FS_SEL_2G		0x00
#define	ACCEL_FS_SEL_4G		0x08
#define ACCEL_FS_SEL_8G 	0x10
#define ACCEL_FS_SEL_16G	0x18

#define GYRO_CONFIG		0x1A
#define GYRO_FS_SEL_250DPS	0x00
#define GYRO_FS_SEL_500DPS	0x08
#define GYRO_FS_SEL_1000DPS	0x10
#define GYRO_FS_SEL_2000DPS	0x18

#define PWR_MGMNT_1		0x6B
#define CLOCK_SEL_PLL		0x01

#define G			9.807
#define D2R			180.0/PI						



class MPU9250{
  public:
    MPU9250(int address);
    void begin(String accelRange, String gyroRange);
    void getAccel(double* ax, double* ay, double* az);
    void getGyro(double* gx, double* gy, double* gz);
    //void getMag(double* hx, double* hy, double* hz);
    void getMotion6(double* ax, double* ay, double* az, double* gx, double* gy, double* gz);
  private:
    int _address;
    double _accelScale;
    double _gyroScale;
    void writeRegister(uint8_t subAddress, uint8_t data);
    void readRegisters(uint8_t subAddress, int count, uint8_t* dest);
};

#endif
