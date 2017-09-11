/*
MPU9250.h
Brian R Taylor
brian.taylor@bolderflight.com

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

#ifndef MPU9250_h
#define MPU9250_h

#include "Arduino.h"
#include "i2c_t3.h"  // I2C library
#include "SPI.h"     // SPI library

enum mpu9250_gyro_range
{
    GYRO_RANGE_250DPS,
    GYRO_RANGE_500DPS,
    GYRO_RANGE_1000DPS,
    GYRO_RANGE_2000DPS
};

enum mpu9250_accel_range
{
    ACCEL_RANGE_2G,
    ACCEL_RANGE_4G,
    ACCEL_RANGE_8G,
    ACCEL_RANGE_16G    
};

enum mpu9250_dlpf_bandwidth
{
    DLPF_BANDWIDTH_184HZ,
    DLPF_BANDWIDTH_92HZ,
    DLPF_BANDWIDTH_41HZ,
    DLPF_BANDWIDTH_20HZ,
    DLPF_BANDWIDTH_10HZ,
    DLPF_BANDWIDTH_5HZ
};

class MPU9250{
    public:
        MPU9250(uint8_t address, uint8_t bus);
        MPU9250(uint8_t address, uint8_t bus, i2c_pins pins);
        MPU9250(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups);
        MPU9250(uint8_t csPin);
        MPU9250(uint8_t csPin, SPIClass *Spi);
        int begin(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange);
        int setFilt(mpu9250_dlpf_bandwidth bandwidth, uint8_t SRD);
        int enableInt(bool enable);
        void getAccel(float* ax, float* ay, float* az);
        void getGyro(float* gx, float* gy, float* gz);
        void getMag(float* hx, float* hy, float* hz);
        void getTemp(float *t);
        void getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz);
        void getMotion7(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* t);
        void getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz);
        void getMotion10(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz, float* t);

        void getAccelCounts(int16_t* ax, int16_t* ay, int16_t* az);
        void getGyroCounts(int16_t* gx, int16_t* gy, int16_t* gz);
        void getMagCounts(int16_t* hx, int16_t* hy, int16_t* hz);
        void getTempCounts(int16_t* t);
        void getMotion6Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);
        void getMotion7Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* t);
        void getMotion9Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz);
        void getMotion10Counts(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz, int16_t* hx, int16_t* hy, int16_t* hz, int16_t* t);
    private:
        uint8_t _address;
        uint8_t _bus;
        i2c_pins _pins;
        i2c_pullup _pullups;
        bool _userDefI2C;
        uint8_t _csPin;
        SPIClass *_spi;
        bool _useSPI;
        bool _useSPIHS;
        float _accelScale;
        float _gyroScale;
        float _magScaleX, _magScaleY, _magScaleZ;
        const float _tempScale = 333.87f;
        const float _tempOffset = 21.0f;

        // SPI constants
        const uint8_t SPI_READ = 0x80;
        const uint32_t SPI_LS_CLOCK = 1000000; // 1 MHz
        const uint32_t SPI_HS_CLOCK = 20000000; // 20 MHz

        // i2c bus frequency
        const uint32_t _i2cRate = 400000;

        // constants
        const float G = 9.807f;
        const float _d2r = 3.14159265359f/180.0f;

        // MPU9250 registers
        const uint8_t ACCEL_OUT = 0x3B;
        const uint8_t GYRO_OUT = 0x43;
        const uint8_t TEMP_OUT = 0x41;
        const uint8_t EXT_SENS_DATA_00 = 0x49;

        const uint8_t ACCEL_CONFIG = 0x1C;
        const uint8_t ACCEL_FS_SEL_2G = 0x00;
        const uint8_t ACCEL_FS_SEL_4G = 0x08;
        const uint8_t ACCEL_FS_SEL_8G = 0x10;
        const uint8_t ACCEL_FS_SEL_16G = 0x18;

        const uint8_t GYRO_CONFIG = 0x1B;
        const uint8_t GYRO_FS_SEL_250DPS = 0x00;
        const uint8_t GYRO_FS_SEL_500DPS = 0x08;
        const uint8_t GYRO_FS_SEL_1000DPS = 0x10;
        const uint8_t GYRO_FS_SEL_2000DPS = 0x18;

        const uint8_t ACCEL_CONFIG2 = 0x1D;
        const uint8_t ACCEL_DLPF_184 = 0x01;
        const uint8_t ACCEL_DLPF_92 = 0x02;
        const uint8_t ACCEL_DLPF_41 = 0x03;
        const uint8_t ACCEL_DLPF_20 = 0x04;
        const uint8_t ACCEL_DLPF_10 = 0x05;
        const uint8_t ACCEL_DLPF_5 = 0x06;

        const uint8_t CONFIG = 0x1A;
        const uint8_t GYRO_DLPF_184 = 0x01;
        const uint8_t GYRO_DLPF_92 = 0x02;
        const uint8_t GYRO_DLPF_41 = 0x03;
        const uint8_t GYRO_DLPF_20 = 0x04;
        const uint8_t GYRO_DLPF_10 = 0x05;
        const uint8_t GYRO_DLPF_5 = 0x06;

        const uint8_t SMPDIV = 0x19;

        const uint8_t INT_PIN_CFG = 0x37;
        const uint8_t INT_ENABLE = 0x38;
        const uint8_t INT_DISABLE = 0x00;
        const uint8_t INT_PULSE_50US = 0x00;
        const uint8_t INT_RAW_RDY_EN = 0x01;

        const uint8_t PWR_MGMNT_1 = 0x6B;
        const uint8_t PWR_RESET = 0x80;
        const uint8_t CLOCK_SEL_PLL = 0x01;

        const uint8_t PWR_MGMNT_2 = 0x6C;
        const uint8_t SEN_ENABLE = 0x00;

        const uint8_t USER_CTRL = 0x6A;
        const uint8_t I2C_MST_EN = 0x20;
        const uint8_t I2C_MST_CLK = 0x0D;
        const uint8_t I2C_MST_CTRL = 0x24;
        const uint8_t I2C_SLV0_ADDR = 0x25;
        const uint8_t I2C_SLV0_REG = 0x26;
        const uint8_t I2C_SLV0_DO = 0x63;
        const uint8_t I2C_SLV0_CTRL = 0x27;
        const uint8_t I2C_SLV0_EN = 0x80;
        const uint8_t I2C_READ_FLAG = 0x80;

        const uint8_t WHO_AM_I = 0x75;

        // AK8963 registers
        const uint8_t AK8963_I2C_ADDR = 0x0C;

        const uint8_t AK8963_HXL = 0x03; 

        const uint8_t AK8963_CNTL1 = 0x0A;
        const uint8_t AK8963_PWR_DOWN = 0x00;
        const uint8_t AK8963_CNT_MEAS1 = 0x12;
        const uint8_t AK8963_CNT_MEAS2 = 0x16;
        const uint8_t AK8963_FUSE_ROM = 0x0F;

        const uint8_t AK8963_CNTL2 = 0x0B;
        const uint8_t AK8963_RESET = 0x01;

        const uint8_t AK8963_ASA = 0x10;

        const uint8_t AK8963_WHO_AM_I = 0x00;

        // transformation matrix
        /* transform the accel and gyro axes to match the magnetometer axes */
        const int16_t tX[3] = {0,  1,  0}; 
        const int16_t tY[3] = {1,  0,  0};
        const int16_t tZ[3] = {0,  0, -1};

        bool writeRegister(uint8_t subAddress, uint8_t data);
        void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest);
        bool writeAK8963Register(uint8_t subAddress, uint8_t data);
        void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest);
        uint8_t whoAmI();
        uint8_t whoAmIAK8963();
};

#endif
