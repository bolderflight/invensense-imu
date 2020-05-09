/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "core/core.h"
#include "mpu9250/mpu9250.h"
#include "remote_test/remote_test.h"
#include "serial_link/serial_link.h"
#include "mcu_hil_defs/mcu_hil_defs.h"
#include <math.h>

/* Test the ability to begin communications over SPI */
bool TestBeginSpi() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  bool status = mpu.Begin();
  return status;
}
/* Test the ability to begin communication over I2C */
bool TestBeginI2c() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  bool status = mpu.Begin();
  return status;
}
/* Test data collection over SPI */

/* Test data collection over I2C */

/* Test accel range, x and y near 0, z near -1 */
bool CheckAccel(Accel accel) {
  float thresh = 0.5f;
  if (fabs(accel.x_g()) > thresh) {
    return false;
  }
  if (fabs(accel.y_g()) > thresh) {
    return false;
  }
  if (fabs(accel.z_g()) > (-1.0f + thresh)) {
    return false;
  }
  if (fabs(accel.z_g()) < (-1.0f - thresh)) {
    return false;
  }
  return true;
}
/* Test all available accel full scale ranges */
bool TestAccelRange(sensors::Mpu9250 &mpu) {
  delay(2);
  if (!mpu.accel_range(sensors::Mpu9250::ACCEL_RANGE_2G)) {
    return false;
  }
  if (!mpu.Read()) {
    return false;
  }
  Imu imu = mpu.imu();
  if (!CheckAccel(imu.accel)) {
    return false;
  }
  delay(2);
  if (!mpu.accel_range(sensors::Mpu9250::ACCEL_RANGE_4G)) {
    return false;
  }
  if (!mpu.Read()) {
    return false;
  }
  imu = mpu.imu();
  if (!CheckAccel(imu.accel)) {
    return false;
  }
  delay(2);
  if (!mpu.accel_range(sensors::Mpu9250::ACCEL_RANGE_8G)) {
    return false;
  }
  if (!mpu.Read()) {
    return false;
  }
  imu = mpu.imu();
  if (!CheckAccel(imu.accel)) {
    return false;
  }
  delay(2);
  if (!mpu.accel_range(sensors::Mpu9250::ACCEL_RANGE_16G)) {
    return false;
  }
  if (!mpu.Read()) {
    return false;
  }
  imu = mpu.imu();
  if (!CheckAccel(imu.accel)) {
    return false;
  }
  return true; 
}
/* Test accel range SPI */
bool TestAccelRangeSpi() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  bool status = mpu.Begin();
  if (!status) {
    return false;
  }
  return true;
}
/* Test accel range I2C */
bool TestAccelRangeI2c() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  bool status = mpu.Begin();
  if (!status) {
    return false;
  }
  return true;
}
// /* Test gyro, all channels near zero */
// bool TestGyro(Gyro gyro) {
//   float thresh
// }


/* Test gyro range SPI */

/* Test gyro range I2C */

/* Test SRD SPI */

/* Test SRD I2C */

/* Test DLPF SPI */

/* Test DLPF I2C */

/* Test rotation SPI */

/* Test rotation I2C */


int main() {
  /* Starting the remote test interface */
  UsbSerialLink link(Serial);
  RemoteTestSlave test(link);
  /* Registering tests */
  test.AddTest(1, TestBeginSpi);
  test.AddTest(2, TestBeginI2c);
  test.AddTest(5, TestAccelRangeSpi);
  test.AddTest(6, TestAccelRangeI2c);
  while (1) {
    /* Check for new tests */
    test.Check();
  }
}
