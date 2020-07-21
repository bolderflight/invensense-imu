/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "mpu9250/mpu9250.h"

/* Mpu9250 object using I2C */
sensors::Mpu9250 mpu9250(&Wire, 0x68);
/* Data acquisition ISR */
void imu_isr() {
  /* Check if data read */
  if (mpu9250.Read()) {
    /* Print data */
     types::Imuf imu = mpu9250.imu();
    // types::Mag mag = mpu9250.mag();
    types::Temperaturef t = mpu9250.die_temperature();
    Serial.print(imu.accel.x.mps2());
    Serial.print("\t");
    Serial.print(imu.accel.y.mps2());
    Serial.print("\t");
    Serial.print(imu.accel.z.mps2());
    Serial.print("\t");
    Serial.print(imu.gyro.x.radps());
    Serial.print("\t");
    Serial.print(imu.gyro.y.radps());
    Serial.print("\t");
    Serial.print(imu.gyro.z.radps());
    Serial.print("\t");
    // Serial.print(mag.x_ut());
    // Serial.print("\t");
    // Serial.print(mag.y_ut());
    // Serial.print("\t");
    // Serial.print(mag.z_ut());
    // Serial.print("\t");
    Serial.print(t.c());
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
  status = mpu9250.sample_rate_divider(19);
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
  mpu9250.DrdyCallback(4, imu_isr);
  while (1) {}
}
