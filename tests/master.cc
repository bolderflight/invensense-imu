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
  sleep(1);
  /* Remote Test setup */
  HardwareSerial serial("/dev/ttyACM1");
  SerialLink link(serial);
  RemoteTestMaster test(link);
  /* Command test */
  bool result = test.Test(2, 10);
  EXPECT_TRUE(result);
}

