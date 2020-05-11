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

/* Test the ability to begin communication over I2C */
bool BeginI2c() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  bool status = mpu.Begin();
  return status;
}
/* Test the ability to begin communication over SPI */
bool BeginSpi() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  bool status = mpu.Begin();
  return status;
}
/* Test bad I2C config */
bool BeginI2cIncorrect() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, 0x67);
  bool status = mpu.Begin();
  if (status == false) {
    return true;
  } else {
    return false;
  }
}
/* Test bad SPI config */
bool BeginSpiIncorrect() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS + 1);
  bool status = mpu.Begin();
  if (status == false) {
    return true;
  } else {
    return false;
  }
}
/* Test enable / disable interrupt */
volatile int int_count = 0;
/* ISR for testing interrupts */
void IntIsr() {
  int_count++;
}
bool TestInterrupt(sensors::Mpu9250 *mpu) {
  int_count = 0;
  if (!mpu->sample_rate_divider(19)) {
    return false;
  }
  if (!mpu->EnableDrdyInt()) {
    return false;
  }
  /* wait 200 milliseconds */
  delay(200);
  if (!mpu->DisableDrdyInt()) {
    return false;
  }
  delay(200);
  if (int_count != 10) {
    return false;
  }
  return true;
}
/* Interrupt test I2C */
bool I2CInterrupt() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  if (!mpu.Begin()) {
    return false;
  }
  mpu.DrdyCallback(MPU9250_I2C_INT, IntIsr);
  return TestInterrupt(&mpu);
}
/* Interrupt test SPI */
bool SpiInterrupt() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  if (!mpu.Begin()) {
    return false;
  }
  mpu.DrdyCallback(MPU9250_SPI_INT, IntIsr);
  return TestInterrupt(&mpu);
}
/* Rotation test */
bool TestRotation(sensors::Mpu9250 *mpu) {
  /* Get 10 samples of data */
  Imu imu[10];
  unsigned int i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  /* Take the mean of each */
  float mean_x = 0, mean_y = 0, mean_z =0;
  for (unsigned int i = 0; i < 10; i++) {
    mean_x += imu[i].accel.x_g() / 10.0f;
    mean_y += imu[i].accel.y_g() / 10.0f;
    mean_z += imu[i].accel.z_g() / 10.0f;
  }
  /* Check the rotation */
  float tol = 0.1f;
  if (fabs(mean_x) > tol) {
    return false;
  }
  if (fabs(mean_y) > tol) {
    return false;
  }
  if (mean_z > (-1.0f + tol)) {
    return false;
  }
  if (mean_z < (-1.0f - tol)) {
    return false;
  }
  /* Negate Z */
  Eigen::Matrix3f rot = Eigen::Matrix3f::Zero();
  rot(0, 0) = 1.0f;
  rot(1, 1) = 1.0f;
  rot(2, 2) = -1.0f;
  mpu->rotation(rot);
  Eigen::Matrix3f ret = mpu->rotation();
  if (ret != rot) {
    return false;
  }
  /* Get 10 samples of data */
  i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  /* Take the mean of each */
  mean_x = 0, mean_y = 0, mean_z =0;
  for (unsigned int i = 0; i < 10; i++) {
    mean_x += imu[i].accel.x_g() / 10.0f;
    mean_y += imu[i].accel.y_g() / 10.0f;
    mean_z += imu[i].accel.z_g() / 10.0f;
  }
  /* Check the rotation */
  if (fabs(mean_x) > tol) {
    return false;
  }
  if (fabs(mean_y) > tol) {
    return false;
  }
  if (mean_z > (1.0f + tol)) {
    return false;
  }
  if (mean_z < (1.0f - tol)) {
    return false;
  }
  /* Swap X and Z */
  rot = Eigen::Matrix3f::Zero();
  rot(0, 2) = 1.0f;
  rot(1, 1) = 1.0f;
  rot(2, 0) = 1.0f;
  mpu->rotation(rot);
  ret = mpu->rotation();
  if (ret != rot) {
    return false;
  }
  /* Get 10 samples of data */
  i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  /* Take the mean of each */
  mean_x = 0, mean_y = 0, mean_z =0;
  for (unsigned int i = 0; i < 10; i++) {
    mean_x += imu[i].accel.x_g() / 10.0f;
    mean_y += imu[i].accel.y_g() / 10.0f;
    mean_z += imu[i].accel.z_g() / 10.0f;
  }
  /* Check the rotation */
  if (fabs(mean_z) > tol) {
    return false;
  }
  if (fabs(mean_y) > tol) {
    return false;
  }
  if (mean_x > (-1.0f + tol)) {
    return false;
  }
  if (mean_x < (-1.0f - tol)) {
    return false;
  }
  /* Negate X */
  rot = Eigen::Matrix3f::Zero();
  rot(0, 2) = -1.0f;
  rot(1, 1) = 1.0f;
  rot(2, 0) = 1.0f;
  mpu->rotation(rot);
  ret = mpu->rotation();
  if (ret != rot) {
    Serial.println("Rotation matrix mis-match");
  }
  /* Get 10 samples of data */
  i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  /* Take the mean of each */
  mean_x = 0, mean_y = 0, mean_z =0;
  for (unsigned int i = 0; i < 10; i++) {
    mean_x += imu[i].accel.x_g() / 10.0f;
    mean_y += imu[i].accel.y_g() / 10.0f;
    mean_z += imu[i].accel.z_g() / 10.0f;
  }
  /* Check the rotation */
  if (fabs(mean_z) > tol) {
    return false;
  }
  if (fabs(mean_y) > tol) {
    return false;
  }
  if (mean_x > (1.0f + tol)) {
    return false;
  }
  if (mean_x < (1.0f - tol)) {
    return false;
  }
  /* Swap Y and Z */
  rot = Eigen::Matrix3f::Zero();
  rot(0, 0) = 1.0f;
  rot(1, 2) = 1.0f;
  rot(2, 1) = 1.0f;
  mpu->rotation(rot);
  ret = mpu->rotation();
  if (ret != rot) {
    return false;
  }
  /* Get 10 samples of data */
  i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  /* Take the mean of each */
  mean_x = 0, mean_y = 0, mean_z =0;
  for (unsigned int i = 0; i < 10; i++) {
    mean_x += imu[i].accel.x_g() / 10.0f;
    mean_y += imu[i].accel.y_g() / 10.0f;
    mean_z += imu[i].accel.z_g() / 10.0f;
  }
  /* Check the rotation */
  if (fabs(mean_x) > tol) {
    return false;
  }
  if (fabs(mean_z) > tol) {
    return false;
  }
  if (mean_y > (-1.0f + tol)) {
    return false;
  }
  if (mean_y < (-1.0f - tol)) {
    return false;
  }
  /* Negate Y */
  rot = Eigen::Matrix3f::Zero();
  rot(0, 0) = 1.0f;
  rot(1, 2) = -1.0f;
  rot(2, 1) = 1.0f;
  mpu->rotation(rot);
  ret = mpu->rotation();
  if (ret != rot) {
    return false;
  }
  /* Get 10 samples of data */
  i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  /* Take the mean of each */
  mean_x = 0, mean_y = 0, mean_z =0;
  for (unsigned int i = 0; i < 10; i++) {
    mean_x += imu[i].accel.x_g() / 10.0f;
    mean_y += imu[i].accel.y_g() / 10.0f;
    mean_z += imu[i].accel.z_g() / 10.0f;
  }
  /* Check the rotation */
  if (fabs(mean_x) > tol) {
    return false;
  }
  if (fabs(mean_z) > tol) {
    return false;
  }
  if (mean_y > (1.0f + tol)) {
    return false;
  }
  if (mean_y < (1.0f - tol)) {
    return false;
  }
  return true;
}
/* Rotation test I2C */
bool I2cRotation() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  if (!mpu.Begin()) {
    return false;
  }
  return TestRotation(&mpu);
}
/* Rotation test SPI */
bool SpiRotation() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  if (!mpu.Begin()) {
    return false;
  }
  return TestRotation(&mpu);
}
/* Test accel range */
bool TestAccelRange(sensors::Mpu9250 *mpu) {
  /* Set and test accel range */
  sensors::Mpu9250::AccelRange set_range = sensors::Mpu9250::ACCEL_RANGE_2G;
  if (!mpu->accel_range(set_range)) {
    return false;
  }
  sensors::Mpu9250::AccelRange ret_range = mpu->accel_range();
  if (ret_range != set_range) {
    return false;
  }
  unsigned int i = 0;
  Imu imu[10];
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  float mean = 0;
  for (unsigned int i = 0; i < 10; i++) {
    mean += imu[i].accel.z_g() / 10.0f;
  }
  float tol = 0.1f;
  if (mean > (-1.0f + tol)) {
    return false;
  }
  if (mean < (-1.0f - tol)) {
    return false;
  }
  /* Range 4G */
  set_range = sensors::Mpu9250::ACCEL_RANGE_4G;
  if (!mpu->accel_range(set_range)) {
    return false;
  }
  ret_range = mpu->accel_range();
  if (ret_range != set_range) {
    return false;
  }
  i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  mean = 0;
  for (unsigned int i = 0; i < 10; i++) {
    mean += imu[i].accel.z_g() / 10.0f;
  }
  if (mean > (-1.0f + tol)) {
    return false;
  }
  if (mean < (-1.0f - tol)) {
    return false;
  }
    /* Range 8G */
  set_range = sensors::Mpu9250::ACCEL_RANGE_8G;
  if (!mpu->accel_range(set_range)) {
    return false;
  }
  ret_range = mpu->accel_range();
  if (ret_range != set_range) {
    return false;
  }
  i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  mean = 0;
  for (unsigned int i = 0; i < 10; i++) {
    mean += imu[i].accel.z_g() / 10.0f;
  }
  if (mean > (-1.0f + tol)) {
    return false;
  }
  if (mean < (-1.0f - tol)) {
    return false;
  }
  /* Range 16G */
  set_range = sensors::Mpu9250::ACCEL_RANGE_16G;
  if (!mpu->accel_range(set_range)) {
    return false;
  }
  ret_range = mpu->accel_range();
  if (ret_range != set_range) {
    return false;
  }
  i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  mean = 0;
  for (unsigned int i = 0; i < 10; i++) {
    mean += imu[i].accel.z_g() / 10.0f;
  }
  if (mean > (-1.0f + tol)) {
    return false;
  }
  if (mean < (-1.0f - tol)) {
    return false;
  }
  return true;
}
/* Accel range test I2C */
bool I2cAccelRange() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  if (!mpu.Begin()) {
    return false;
  }
  return TestAccelRange(&mpu);
}
/* Accel range test SPI */
bool SpiAccelRange() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  if (!mpu.Begin()) {
    return false;
  }
  return TestAccelRange(&mpu);
}
/* Test gyro range */
bool TestGyroRange(sensors::Mpu9250 *mpu) {
  /* Set and test gyro range */
  sensors::Mpu9250::GyroRange set_range = sensors::Mpu9250::GYRO_RANGE_250DPS;
  if (!mpu->gyro_range(set_range)) {
    return false;
  }
  sensors::Mpu9250::GyroRange ret_range = mpu->gyro_range();
  if (ret_range != set_range) {
    return false;
  }
  unsigned int i = 0;
  Imu imu[10];
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  float mean_x = 0, mean_y = 0, mean_z = 0;
  for (unsigned int i = 0; i < 10; i++) {
    mean_x += imu[i].gyro.x_dps() / 10.0f;
    mean_y += imu[i].gyro.y_dps() / 10.0f;
    mean_z += imu[i].gyro.z_dps() / 10.0f;
  }
  float tol = 3.0f;
  if (fabs(mean_x) > tol) {
    return false;
  }
  if (fabs(mean_y) > tol) {
    return false;
  }
  if (fabs(mean_z) > tol) {
    return false;
  }
  /* Range 500 DPS */
  set_range = sensors::Mpu9250::GYRO_RANGE_500DPS;
  if (!mpu->gyro_range(set_range)) {
    return false;
  }
  ret_range = mpu->gyro_range();
  if (ret_range != set_range) {
    return false;
  }
  i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  mean_x = 0, mean_y = 0, mean_z = 0;
  for (unsigned int i = 0; i < 10; i++) {
    mean_x += imu[i].gyro.x_dps() / 10.0f;
    mean_y += imu[i].gyro.y_dps() / 10.0f;
    mean_z += imu[i].gyro.z_dps() / 10.0f;
  }
  if (fabs(mean_x) > tol) {
    return false;
  }
  if (fabs(mean_y) > tol) {
    return false;
  }
  if (fabs(mean_z) > tol) {
    return false;
  }
  /* Range 1000 DPS */
  set_range = sensors::Mpu9250::GYRO_RANGE_1000DPS;
  if (!mpu->gyro_range(set_range)) {
    return false;
  }
  ret_range = mpu->gyro_range();
  if (ret_range != set_range) {
    return false;
  }
  i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  mean_x = 0, mean_y = 0, mean_z = 0;
  for (unsigned int i = 0; i < 10; i++) {
    mean_x += imu[i].gyro.x_dps() / 10.0f;
    mean_y += imu[i].gyro.y_dps() / 10.0f;
    mean_z += imu[i].gyro.z_dps() / 10.0f;
  }
  if (fabs(mean_x) > tol) {
    return false;
  }
  if (fabs(mean_y) > tol) {
    return false;
  }
  if (fabs(mean_z) > tol) {
    return false;
  }
  /* Range 2000 DPS */
  set_range = sensors::Mpu9250::GYRO_RANGE_2000DPS;
  if (!mpu->gyro_range(set_range)) {
    return false;
  }
  ret_range = mpu->gyro_range();
  if (ret_range != set_range) {
    return false;
  }
  i = 0;
  while (i < 10) {
    if (mpu->Read()) {
      imu[i++] = mpu->imu();
    }
  }
  mean_x = 0, mean_y = 0, mean_z = 0;
  for (unsigned int i = 0; i < 10; i++) {
    mean_x += imu[i].gyro.x_dps() / 10.0f;
    mean_y += imu[i].gyro.y_dps() / 10.0f;
    mean_z += imu[i].gyro.z_dps() / 10.0f;
  }
  if (fabs(mean_x) > tol) {
    return false;
  }
  if (fabs(mean_y) > tol) {
    return false;
  }
  if (fabs(mean_z) > tol) {
    return false;
  }
  return true;
}
/* Gyro range test I2C */
bool I2cGyroRange() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  if (!mpu.Begin()) {
    return false;
  }
  return TestGyroRange(&mpu);
}
/* Gyro range test SPI */
bool SpiGyroRange() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  if (!mpu.Begin()) {
    return false;
  }
  return TestGyroRange(&mpu);
}
/* Test SRD */
unsigned int t1, t2;
unsigned int elapsed_time[10];
volatile unsigned int timing_count = 0, array_count = 0;;
void TimingIsr() {
  t2 = micros();
  if (timing_count > 5) {
    if (array_count < 10) {
      elapsed_time[array_count++] = t2 - t1;
    }
  }
  t1 = t2;
  timing_count++;
}
bool TestSrd(sensors::Mpu9250 *mpu) {
  for (unsigned int k = 0; k < 256; k++) {
    float period_us = ((float) k + 1.0f) / 1000.0f * 1e6;
    bool status = mpu->sample_rate_divider(k);
    mpu->EnableDrdyInt();
    while (array_count < 10) {
      delay(10);
    }
    mpu->DisableDrdyInt();
    timing_count = 0;
    array_count = 0;
    float mean = 0;
    for (unsigned int i = 0; i < 10; i++) {
      mean += elapsed_time[i] / 10.0f;
    }
    float tol = 200.0f;
    if ((fabs(mean - period_us) > tol) || (!status)) {
      return false;
    }
  }
  return true;
}
/* SRD test I2C */
bool I2cSrd() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  if (!mpu.Begin()) {
    return false;
  }
  mpu.DrdyCallback(MPU9250_I2C_INT, TimingIsr);
  return TestSrd(&mpu);
}
/* SRD test SPI */
bool SpiSrd() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  if (!mpu.Begin()) {
    return false;
  }
  mpu.DrdyCallback(MPU9250_SPI_INT, TimingIsr);
  return TestSrd(&mpu);
}
/* Test DLPF */
bool TestDlpf(sensors::Mpu9250 *mpu) {
  /* 184Hz DLPF */
  sensors::Mpu9250::DlpfBandwidth set_bandwidth = sensors::Mpu9250::DLPF_BANDWIDTH_184HZ;
  bool status = mpu->dlpf_bandwidth(set_bandwidth);
  if (!status) {
    return false;
  }
  sensors::Mpu9250::DlpfBandwidth ret_bandwidth = mpu->dlpf_bandwidth();
  if (set_bandwidth != ret_bandwidth) {
    return false;
  }
  /* 92Hz DLPF */
  set_bandwidth = sensors::Mpu9250::DLPF_BANDWIDTH_92HZ;
  status = mpu->dlpf_bandwidth(set_bandwidth);
  if (!status) {
    return false;
  }
  ret_bandwidth = mpu->dlpf_bandwidth();
  if (set_bandwidth != ret_bandwidth) {
    return false;
  }
  /* 41Hz DLPF */
  set_bandwidth = sensors::Mpu9250::DLPF_BANDWIDTH_41HZ;
  status = mpu->dlpf_bandwidth(set_bandwidth);
  if (!status) {
    return false;
  }
  ret_bandwidth = mpu->dlpf_bandwidth();
  if (set_bandwidth != ret_bandwidth) {
    return false;
  }
  /* 20Hz DLPF */
  set_bandwidth = sensors::Mpu9250::DLPF_BANDWIDTH_20HZ;
  status = mpu->dlpf_bandwidth(set_bandwidth);
  if (!status) {
    return false;
  }
  ret_bandwidth = mpu->dlpf_bandwidth();
  if (set_bandwidth != ret_bandwidth) {
    return false;
  }
  /* 10Hz DLPF */
  set_bandwidth = sensors::Mpu9250::DLPF_BANDWIDTH_10HZ;
  status = mpu->dlpf_bandwidth(set_bandwidth);
  if (!status) {
    return false;
  }
  ret_bandwidth = mpu->dlpf_bandwidth();
  if (set_bandwidth != ret_bandwidth) {
    return false;
  }
  /* 5Hz DLPF */
  set_bandwidth = sensors::Mpu9250::DLPF_BANDWIDTH_5HZ;
  status = mpu->dlpf_bandwidth(set_bandwidth);
  if (!status) {
    return false;
  }
  ret_bandwidth = mpu->dlpf_bandwidth();
  if (set_bandwidth != ret_bandwidth) {
    return false;
  }
  return true;
}
/* DLPF test I2C */
bool I2cDlpf() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  if (!mpu.Begin()) {
    return false;
  }
  return TestDlpf(&mpu);
}
/* DLPF test SPI */
bool SpiDlpf() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  if (!mpu.Begin()) {
    return false;
  }
  return TestDlpf(&mpu);
}
/* Test read */
bool TestRead(sensors::Mpu9250 *mpu) {
  if (!mpu->sample_rate_divider(4)) {
    return false;
  }
  unsigned int i = 0;
  Imu imu[10];
  Mag mag[10];
  Temperature temp[10];
  while (i < 10) {
    if (mpu->Read()) {
      imu[i] = mpu->imu();
      mag[i] = mpu->mag();
      temp[i] = mpu->die_temperature();
      i++;
    }
  }
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, hx = 0, hy = 0, hz = 0, t = 0;
  for (unsigned int i = 0; i < 10; i++) {
    ax += imu[i].accel.x_g() / 10.0f;
    ay += imu[i].accel.y_g() / 10.0f;
    az += imu[i].accel.z_g() / 10.0f;
    gx += imu[i].gyro.x_dps() / 10.0f;
    gy += imu[i].gyro.y_dps() / 10.0f;
    gz += imu[i].gyro.z_dps() / 10.0f;
    hx += mag[i].x_t() / 10.0f;
    hy += mag[i].y_t() / 10.0f;
    hz += mag[i].z_t() / 10.0f;
    t += temp[i].c() / 10.0f;
  }
  float accel_tol = 0.1f;
  float gyro_tol = 3.0f;
  if ((fabs(ax) == 0.0f) || (fabs(ax) > accel_tol)) {
    return false;
  }
  if ((fabs(ay) == 0.0f) || (fabs(ay) > accel_tol)) {
    return false;
  }
  if ((az > (1.0f + accel_tol)) || (az < (-1.0f - accel_tol))) {
    return false;
  }
  if ((fabs(gx) == 0.0f) || (fabs(gx) > gyro_tol)) {
    return false;
  }
  if ((fabs(gy) == 0.0f) || (fabs(gy) > gyro_tol)) {
    return false;
  }
  if ((fabs(gz) == 0.0f) || (fabs(gz) > gyro_tol)) {
    return false;
  }
  if (fabs(hx) == 0.0f) {
    return false;
  }
  if (fabs(hy) == 0.0f) {
    return false;
  }
  if (fabs(hz) == 0.0f) {
    return false;
  }
  if (t < 20.0f) {
    return false;
  }
  if (t > 35.0f) {
    return false;
  }
  return true;
}
/* Read test I2C */
bool I2cRead() {
  sensors::Mpu9250 mpu(&MPU9250_I2C, MPU9250_I2C_ADDR);
  if (!mpu.Begin()) {
    return false;
  }
  return TestRead(&mpu);
}
/* Read test SPI */
bool SpiRead() {
  sensors::Mpu9250 mpu(&MPU9250_SPI, MPU9250_SPI_CS);
  if (!mpu.Begin()) {
    return false;
  }
  return TestRead(&mpu);
}
int main() {
  /* Starting the remote test interface */
  UsbSerialLink link(Serial);
  RemoteTestSlave test(link);
  /* Registering tests */
  test.AddTest(1, BeginI2c);
  test.AddTest(2, BeginSpi);
  test.AddTest(3, BeginI2cIncorrect);
  test.AddTest(4, BeginSpiIncorrect);
  test.AddTest(5, I2CInterrupt);
  test.AddTest(6, SpiInterrupt);
  test.AddTest(7, I2cRotation);
  test.AddTest(8, SpiRotation);
  test.AddTest(9, I2cAccelRange);
  test.AddTest(10, SpiAccelRange);
  test.AddTest(11, I2cGyroRange);
  test.AddTest(12, SpiGyroRange);
  test.AddTest(13, I2cSrd);
  test.AddTest(14, SpiSrd);
  test.AddTest(15, I2cDlpf);
  test.AddTest(16, SpiDlpf);
  test.AddTest(17, I2cRead);
  test.AddTest(18, SpiRead);
  while (1) {
    /* Check for new tests */
    test.Check();
  }
}
