/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "hardware_serial/hardware_serial.h"
#include "remote_test/remote_test.h"
#include "serial_link/serial_link.h"
#include "gtest/gtest.h"

TEST(Mpu9250, BeginI2c) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(1, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, BeginSpi) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(2, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, BeginI2cIncorrect) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(3, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, BeginSpiIncorrect) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(4, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, I2CInterrupt) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(5, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, SpiInterrupt) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(6, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, I2cRotation) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(7, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, SpiRotation) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(8, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, I2cAccelRange) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(9, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, SpiAccelRange) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(10, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, I2cGyroRange) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(11, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, SpiGyroRange) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(12, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, I2cSrd) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(13, 3600);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, SpiSrd) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(14, 3600);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, I2cDlpf) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(15, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, SpiDlpf) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(16, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, I2cRead) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(17, 10);
  EXPECT_TRUE(result);
}

TEST(Mpu9250, SpiRead) {
  /* Reset mcu */
  system("mcu_reset");
  /* Remote Test setup */
  HardwareSerial serial("/dev/uart");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(18, 10);
  EXPECT_TRUE(result);
}
