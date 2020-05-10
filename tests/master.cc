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

TEST(Mpu9250, BeginSpi) {
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

TEST(Mpu9250, BeginI2c) {
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

TEST(Mpu9250,AccelRangeSpi) {
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

TEST(Mpu9250, AccelRangeI2c) {
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

TEST(Mpu9250,GyroRangeSpi) {
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

TEST(Mpu9250, GyroRangeI2c) {
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

