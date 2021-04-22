# Changelog

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
