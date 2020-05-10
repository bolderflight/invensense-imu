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
  float thresh = 0.05f;
  if (fabs(accel.x_g()) > thresh) {
    return false;
  }
  if (fabs(accel.y_g()) > thresh) {
    return false;
  }
  if (fabs(accel.z_g()) > (1.0f + thresh)) {
    return false;
  }
  if (fabs(accel.z_g()) < (-1.0f - thresh)) {
    return false;
  }
  return true;
}
/* Test all available accel full scale ranges */
bool TestAccelRange(sensors::Mpu9250 *mpu) {
  delay(2);
  if (!mpu->accel_range(sensors::Mpu9250::ACCEL_RANGE_2G)) {
    return false;
  }
  delay(2);
  if (!mpu->Read()) {
    return false;
  }
  Imu imu = mpu->imu();
  if (!CheckAccel(imu.accel)) {
    return false;
  }
  delay(2);
  if (!mpu->accel_range(sensors::Mpu9250::ACCEL_RANGE_4G)) {
    return false;
  }
  delay(2);
  if (!mpu->Read()) {
    return false;
  }
  imu = mpu->imu();
  if (!CheckAccel(imu.accel)) {
    return false;
  }
  delay(2);
  if (!mpu->accel_range(sensors::Mpu9250::ACCEL_RANGE_8G)) {
    return false;
  }
  delay(2);
  if (!mpu->Read()) {
    return false;
  }
  imu = mpu->imu();
  if (!CheckAccel(imu.accel)) {
    return false;
  }
  delay(2);
  if (!mpu->accel_range(sensors::Mpu9250::ACCEL_RANGE_16G)) {
    return false;
  }
  delay(2);
  if (!mpu->Read()) {
    return false;
  }
  imu = mpu->imu();
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
  return TestAccelRange(&mpu);
}
/* Test accel range I2C */
bool TestAccelRangeI2c() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  bool status = mpu.Begin();
  if (!status) {
    return false;
  }
  return TestAccelRange(&mpu);
}
/* Test gyro, all channels near zero */
bool CheckGyro(Gyro gyro) {
  float thresh = 3.0f;
  if (fabs(gyro.x_dps()) > thresh) {
    return false;
  }
  if (fabs(gyro.y_dps()) > thresh) {
    return false;
  }
  if (fabs(gyro.z_dps()) > thresh) {
    return false;
  }
  return true;
}
/* Test all available gyro full scale ranges */
bool TestGyroRange(sensors::Mpu9250 *mpu) {
  delay(2);
  if (!mpu->gyro_range(sensors::Mpu9250::GYRO_RANGE_250DPS)) {
    return false;
  }
  delay(2);
  if (!mpu->Read()) {
    return false;
  }
  Imu imu = mpu->imu();
  if (!CheckGyro(imu.gyro)) {
    return false;
  }
  delay(2);
  if (!mpu->gyro_range(sensors::Mpu9250::GYRO_RANGE_500DPS)) {
    return false;
  }
  delay(2);
  if (!mpu->Read()) {
    return false;
  }
  imu = mpu->imu();
  if (!CheckGyro(imu.gyro)) {
    return false;
  }
  delay(2);
  if (!mpu->gyro_range(sensors::Mpu9250::GYRO_RANGE_1000DPS)) {
    return false;
  }
  delay(2);
  if (!mpu->Read()) {
    return false;
  }
  imu = mpu->imu();
  if (!CheckGyro(imu.gyro)) {
    return false;
  }
  delay(2);
  if (!mpu->gyro_range(sensors::Mpu9250::GYRO_RANGE_2000DPS)) {
    return false;
  }
  delay(2);
  if (!mpu->Read()) {
    return false;
  }
  imu = mpu->imu();
  if (!CheckGyro(imu.gyro)) {
    return false;
  }
  return true; 
}
/* Test gyro range SPI */
bool TestGyroRangeSpi() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  bool status = mpu.Begin();
  if (!status) {
    return false;
  }
  return TestGyroRange(&mpu);
}
/* Test gyro range I2C */
bool TestGyroRangeI2c() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  bool status = mpu.Begin();
  if (!status) {
    return false;
  }
  return TestGyroRange(&mpu);
}
/* Test SRD SPI */

/* Test SRD I2C */

/* Test DLPF */
bool TestDlpf(sensors::Mpu9250 *mpu) {
  delay(2);
  if (!mpu->dlpf_bandwidth(sensors::Mpu9250::DLPF_BANDWIDTH_184HZ)) {
    return false;
  }
  delay(2);
  if (!mpu->dlpf_bandwidth(sensors::Mpu9250::DLPF_BANDWIDTH_92HZ)) {
    return false;
  }
  delay(2);
  if (!mpu->dlpf_bandwidth(sensors::Mpu9250::DLPF_BANDWIDTH_41HZ)) {
    return false;
  }
  delay(2);
  if (!mpu->dlpf_bandwidth(sensors::Mpu9250::DLPF_BANDWIDTH_20HZ)) {
    return false;
  }
  delay(2);
  if (!mpu->dlpf_bandwidth(sensors::Mpu9250::DLPF_BANDWIDTH_10HZ)) {
    return false;
  }
  delay(2);
  if (!mpu->dlpf_bandwidth(sensors::Mpu9250::DLPF_BANDWIDTH_5HZ)) {
    return false;
  }
  return true;
}

/* Test DLPF SPI */
bool TestDlpfSpi() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  bool status = mpu.Begin();
  if (!status) {
    return false;
  }
  return TestDlpf(&mpu);
}
/* Test DLPF I2C */
bool TestDlpfI2c() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  bool status = mpu.Begin();
  if (!status) {
    return false;
  }
  return TestDlpf(&mpu);
}
/* Test rotation */
bool TestRotation(sensors::Mpu9250 *mpu) {
  float thresh = 0.05f;
  Eigen::Matrix3f rot = Eigen::Matrix3f::Zero();
  /* Check the default rotation */
  if (!mpu->Read()) {
    return false;
  }
  Imu imu = mpu->imu();
  if (fabs(imu.accel.x_g()) > thresh) {
    return false;
  }
  if (fabs(imu.accel.y_g()) > thresh) {
    return false;
  }
  if (fabs(imu.accel.z_g()) > (1.0f + thresh)) {
    return false;
  }
  if (fabs(imu.accel.z_g()) < (1.0f - thresh)) {
    return false;
  }
  // /* Negate Z */
  // rot(0, 0) = 1.0f;
  // rot(1, 1) = 1.0f;
  // rot(2, 2) = -1.0f;
  // if (!mpu->Read()) {
  //   return false;
  // }
  // imu = mpu->imu();
  // if (fabs(imu.accel.x_g()) > thresh) {
  //   return false;
  // }
  // if (fabs(imu.accel.y_g()) > thresh) {
  //   return false;
  // }
  // if (fabs(imu.accel.z_g()) > (-1.0f + thresh)) {
  //   return false;
  // }
  // if (fabs(imu.accel.z_g()) < (-1.0f - thresh)) {
  //   return false;
  // }
  /* Swap X and Z */
  delay(2);
  rot = Eigen::Matrix3f::Zero();
  rot(0, 2) = 1.0f;
  rot(1, 1) = 1.0f;
  rot(2, 0) = 1.0f;
  mpu->rotation(rot);
  delay(2);
  if (!mpu->Read()) {
    return false;
  }
  imu = mpu->imu();
  if (fabs(imu.accel.z_g()) > thresh) {
    return false;
  }
  if (fabs(imu.accel.y_g()) > thresh) {
    return false;
  }
  if (fabs(imu.accel.x_g()) > (1.0f + thresh)) {
    return false;
  }
  if (fabs(imu.accel.x_g()) < (1.0f - thresh)) {
    return false;
  }
  // /* Negate X */
  // delay(2);
  // rot = Eigen::Matrix3f::Zero();
  // rot(0, 2) = -1.0f;
  // rot(1, 1) = 1.0f;
  // rot(2, 0) = 1.0f;
  // mpu->rotation(rot);
  // delay(2);
  // if (!mpu->Read()) {
  //   return false;
  // }
  // imu = mpu->imu();
  // if (fabs(imu.accel.z_g()) > thresh) {
  //   return false;
  // }
  // if (fabs(imu.accel.y_g()) > thresh) {
  //   return false;
  // }
  // if (fabs(imu.accel.x_g()) > (-1.0f + thresh)) {
  //   return false;
  // }
  // if (fabs(imu.accel.x_g()) < (-1.0f - thresh)) {
  //   return false;
  // }
  /* Swap Y and Z */
  delay(2);
  rot = Eigen::Matrix3f::Zero();
  rot(0, 0) = 1.0f;
  rot(1, 2) = 1.0f;
  rot(2, 1) = 1.0f;
  mpu->rotation(rot);
  delay(2);
  if (!mpu->Read()) {
    return false;
  }
  imu = mpu->imu();
  if (fabs(imu.accel.x_g()) > thresh) {
    return false;
  }
  if (fabs(imu.accel.z_g()) > thresh) {
    return false;
  }
  if (fabs(imu.accel.y_g()) > (1.0f + thresh)) {
    return false;
  }
  if (fabs(imu.accel.y_g()) < (1.0f - thresh)) {
    return false;
  }
  // /* Negate Y */
  // delay(2);
  // rot = Eigen::Matrix3f::Zero();
  // rot(0, 0) = 1.0f;
  // rot(1, 2) = -1.0f;
  // rot(2, 1) = 1.0f;
  // mpu->rotation(rot);
  // delay(2);
  // if (!mpu->Read()) {
  //   return false;
  // }
  // imu = mpu->imu();
  // if (fabs(imu.accel.x_g()) > thresh) {
  //   return false;
  // }
  // if (fabs(imu.accel.z_g()) > thresh) {
  //   return false;
  // }
  // if (fabs(imu.accel.y_g()) > (-1.0f + thresh)) {
  //   return false;
  // }
  // if (fabs(imu.accel.y_g()) < (-1.0f - thresh)) {
  //   return false;
  // }
  return true;
}

/* Test rotation SPI */
bool TestRotSpi() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  bool status = mpu.Begin();
  if (!status) {
    return false;
  }
  return TestRotation(&mpu);
}
/* Test rotation I2C */
bool TestRotI2c() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  bool status = mpu.Begin();
  if (!status) {
    return false;
  }
  return TestRotation(&mpu);
}

int main() {
  /* Starting the remote test interface */
  UsbSerialLink link(Serial);
  RemoteTestSlave test(link);
  /* Registering tests */
  test.AddTest(1, TestBeginSpi);
  test.AddTest(2, TestBeginI2c);
  test.AddTest(5, TestAccelRangeSpi);
  test.AddTest(6, TestAccelRangeI2c);
  test.AddTest(7, TestGyroRangeSpi);
  test.AddTest(8, TestGyroRangeI2c);
  test.AddTest(11, TestDlpfSpi);
  test.AddTest(12, TestDlpfI2c);
  test.AddTest(13, TestRotSpi);
  test.AddTest(14, TestRotI2c);
  while (1) {
    /* Check for new tests */
    test.Check();
  }
}
