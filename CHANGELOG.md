# Changelog

## v5.6.0
- Instead of inputting an I2C address, use an enum of the two potential addresses

## v5.5.0
- Updating to include a default constructor

## v5.4.1
- Removing some unneeded ARDUINO if defs

## v5.4.0
- Pulling in Eigen for vector output of accel, gyro, mag

## v5.3.3
- Fixed typos between README and header regarding dlpf_bandwidth

## v5.3.2
- Updating to pull in Units for deg -> rad and g -> m/s/s

## v5.3.1
- Updating to pull in mcu-support repo for cmake, lib, ld, and tools libs

## v5.3.0
- Removing the tools folder to pass the Arduino linting, these will need to be installed on the system seperately for CMake builds

## v5.2.0
- Added FIFO buffer support for accel and gyro data

## v5.1.0
- Added Wake On Motion (WOM) capability and an example

## v5.0.0
- Updated directory structure to support Arduino in addition to CMake builds
- Added an Arduino example
- Updated README

## v4.2.1
- Updated for Teensy 4.1

## v4.2.0
- Updated to imu v2.2.0

## v4.1.0
- Updated to imu v2.1.0

## v4.0.3
- Updated to imu v2.0.3 and eigen v2.0.0

## v4.0.2
- Toggle the CS pin during Begin

## v4.0.1
- Updated to IMU v2.0.2

## v4.0.0
- Edited to conform to IMU interface

## v3.1.2
- Found a better approach to resetting the *new_mag_data* flag.

## v3.1.1
- Fixed an issue where the IMU could return false on *Read*, but *new_mag_data* would return true from the last good read.

## v3.1.0
- Added *new_mag_data* getter to indicate whether new magnetometer data has been read
- Added logic to get the magnetometer status byte in addition to the other magnetometer data to check for whether new data was received
- Check for mag sensor overflow, in which case we set *new_mag_data* to false
- Only update class-stored mag data if valid new data received
- Updated README

## v3.0.0
- Updated to namespace *bfs*
- Updated to support units v3.1.0

## v2.0.1
- Updated to support core v2.0.4 and units v2.0.0
- Updated sources to pull from github

## v2.0.0
- Updating to better match style guide and make clear which methods are requesting the sensor to do something and which are simply acting like a class data member
- Updated README with additional supported processors and updated methods

## v1.0.4
- Updated to support core v2.0.3 which switches Teensy 3.x I2C to TwoWire instead of i2c_t3

## v1.0.3
- Updated cmake, lib, and ld to support Teensy4.x

## v1.0.2
- Updated CONTRIBUTING
- Updated *fetch_content* from ssh to https to enable public access
- Updated *flash_mcu.cmake* to use local loader

## v1.0.1
- Updated license to MIT.
- Used git tags on dependencies.

## v1.0.0
- Initial baseline
