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

/* Test accel range SPI */

/* Test accel range I2C */

/* Test gyro range SPI */

/* Test gyro range I2C */

/* Test SRD SPI */

/* Test SRD I2C */

/* Test DLPF SPI */

/* Test DLPF I2C */


int main() {
  /* Starting the remote test interface */
  UsbSerialLink link(Serial);
  RemoteTestSlave test(link);
  /* Registering tests */
  test.AddTest(1, TestBeginSpi);
  test.AddTest(2, TestBeginI2c);
  while (1) {
    /* Check for new tests */
    test.Check();
  }
}
