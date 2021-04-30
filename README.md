# mpu9250
This library communicates with [InvenSense MPU-9250 and MPU-9255](https://invensense.tdk.com/products/motion-tracking/9-axis/mpu-9250/) Inertial Measurement Units (IMUs).
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

# Description
The InvenSense MPU-9250 is a System in Package (SiP) that combines two chips: the MPU-6500 three-axis gyroscope and three-axis accelerometer; and the AK8963 three-axis magnetometer. The MPU-9250 supports I2C, up to 400 kHz, and SPI communication, up to 1 MHz for register setup and 20 MHz for data reading. The following selectable full scale sensor ranges are available:

| Gyroscope Full Scale Range | Accelerometer Full Scale Range | Magnetometer Full Scale Range |
| --- | --- | ---  |
| +/- 250 deg/s  | +/- 2g  | +/- 4800 uT |
| +/- 500 deg/s  | +/- 4g  | |
| +/- 1000 deg/s | +/- 8g  | |
| +/- 2000 deg/s | +/- 16g | |

The MPU-9250 samples the gyros, accelerometers, and magnetometers with 16 bit analog to digital converters. It also features programmable digital filters, a precision clock, an embedded temperature sensor, and programmable interrupts.

## Installation
CMake is used to build this library, which is exported as a library target called *mpu9250*. The header is added as:

```
#include "mpu9250/mpu9250.h"
```

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and example executables called *i2c_example* and *spi_example*. The example executable source files are located at *examples/i2c.cc* and *examples/spi.cc*. Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64
   * IMXRT1062_T40
   * IMXRT1062_T41

These are known to work with the same packages used in Teensy products. Also switching packages is known to work well, as long as it's only a package change.

The *i2c_example* and *spi_example* targets create executables for communicating with the sensor using I2C or SPI communication, respectively. Each target also has a *_hex* for creating the hex file to upload to the microcontroller. 

## Namespace
This library is within the namespace *bfs*.

## Methods
This driver conforms to the [IMU interface](https://github.com/bolderflight/imu); please refer to those documents for information on the *ImuConfig* and *ImuData* structs.

**bool Init(const ImuConfig &ref)** Initializes communication with the IMU, configures it according to the *ImuConfig* struct, and estimates gyro biases to be removed during *Read*. Note that the bus is not started within init and should be initialized elsewhere. Returns true on successfully initializing and configuring the IMU.

```C++
/* Mpu9250 object */
bfs::Mpu9250 imu;
/* Config */
bfs::ImuConfig config = {
  .bus = &SPI,
  .dev = 24,
  .frame_rate = bfs::RATE_50HZ,
  .accel_bias_mps2 = Eigen::Vector3f::Zero(),
  .mag_bias_ut = Eigen::Vector3f::Zero(),
  .accel_scale = Eigen::Matrix3f::Identity(),
  .mag_scale = Eigen::Matrix3f::Identity(),
  .rotation = Eigen::Matrix3f::Identity()
};
/* Init the bus */
SPI.begin();
/* Initialize and configure IMU */
if (!imu.Init(config)) {
  Serial.println("Error initializing communication with IMU");
  while(1) {}
}
```

**bool Read(ImuData &ast; const ptr)** Reads data from the IMU and passes the data into the *ImuData* struct. Returns true on successfully reading new data.

```C++
/* IMU data */
bfs::ImuData data;
/* Check if data read */
if (imu.Read(&data)) {

}
```
