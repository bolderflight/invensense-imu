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

void yield() {}

bool TestBeginSpi() {
  Mpu9250 mpu(MPU9250_SPI, 2);
  bool status = mpu.Begin();
  return status;
}

int main() {
  /* Starting the remote test interface */
  UsbSerialLink link(Serial);
  RemoteTestSlave test(link);
  /* Registering tests */
  test.AddTest(2, TestBeginSpi);
  while (1) {
    /* Check for new tests */
    test.Check();
  }
}
