/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2020 Bolder Flight Systems
*/

#include "mpu9250/mpu9250.h"

void yield() {}
Mpu9250 imu(SPI, 10);
unsigned int t1, t2;

void imu_isr() {
  if (imu.ReadSensor()) {
    Imu mpu = imu.imu();
    Mag mag = imu.mag();
    Temperature t = imu.die_temperature();
    Serial.print(mpu.accel.x_mss());
    Serial.print("\t");
    Serial.print(mpu.accel.y_mss());
    Serial.print("\t");
    Serial.print(mpu.accel.z_mss());
    Serial.print("\t");
    Serial.print(mpu.gyro.x_dps());
    Serial.print("\t");
    Serial.print(mpu.gyro.y_dps());
    Serial.print("\t");
    Serial.print(mpu.gyro.z_dps());
    Serial.print("\t");
    Serial.print(mag.x_t());
    Serial.print("\t");
    Serial.print(mag.y_t());
    Serial.print("\t");
    Serial.print(mag.z_t());
    Serial.print("\t");
    Serial.print(t.c());
    Serial.print("\n");
  }
}

int main() {

  Serial.begin(115200);
  while(!Serial) {}
  bool status = imu.Begin();
  Serial.println(status);
  status = imu.EnableMag();
  Serial.println(status);
  status = imu.sample_rate_divider(19);
  Serial.println(status);
  status = imu.EnableDrdyInt();
  Serial.println(status);
  imu.DrdyCallback(0, imu_isr);
  while (1) {

  }
}

