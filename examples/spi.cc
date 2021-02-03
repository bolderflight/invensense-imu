/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "mpu9250/mpu9250.h"

/* Mpu9250 object using SPI */
sensors::Mpu9250 mpu9250(&SPI, 10);
/* Data acquisition ISR */
void imu_isr() {
  /* Check if data read */
  if (mpu9250.Read()) {
    /* Print data */
    Serial.print(mpu9250.accel_x_mps2());
    Serial.print("\t");
    Serial.print(mpu9250.accel_y_mps2());
    Serial.print("\t");
    Serial.print(mpu9250.accel_z_mps2());
    Serial.print("\t");
    Serial.print(mpu9250.gyro_x_radps());
    Serial.print("\t");
    Serial.print(mpu9250.gyro_y_radps());
    Serial.print("\t");
    Serial.print(mpu9250.gyro_z_radps());
    Serial.print("\t");
    Serial.print(mpu9250.mag_x_ut());
    Serial.print("\t");
    Serial.print(mpu9250.mag_y_ut());
    Serial.print("\t");
    Serial.print(mpu9250.mag_z_ut());
    Serial.print("\t");
    Serial.print(mpu9250.die_temperature_c());
    Serial.print("\n");
  }
}

int main() {
  /* Serial to display data */
  Serial.begin(115200);
  while(!Serial) {}
  /* Start communicating with MPU-9250 */
  bool status = mpu9250.Begin();
  if (!status) {
    Serial.println("ERROR: unable to communicate with MPU-9250");
    while(1) {}
  }
  /* Set sample rate divider for 50 Hz */
  status = mpu9250.ConfigSrd(19);
  if (!status) {
    Serial.println("ERROR: unable to setup sample rate divider");
    while(1) {}
  }
  /* Enable the data ready interrupt */
  status = mpu9250.EnableDrdyInt();
  if (!status) {
    Serial.println("ERROR: unable to set data ready interrupt");
    while(1) {}
  }
  /* Setup callback for data ready interrupt */
  mpu9250.DrdyCallback(3, imu_isr);
  while (1) {}
}

