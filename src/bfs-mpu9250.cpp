/*
* Brian R Taylor
* brian.taylor@bolderflight.com
* 
* Copyright (c) 2022 Bolder Flight Systems Inc
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the “Software”), to
* deal in the Software without restriction, including without limitation the
* rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
* sell copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
* FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
* IN THE SOFTWARE.
*/

#include "bfs-mpu9250.h"  // NOLINT
#include "utils.h"  // NOLINT

namespace bfs {

bool BfsMpu9250::Init(const Config &config) {
  config_ = config;
  /* Begin and configure the IMU */
  if (!imu_.Begin()) {
    data_.status = SENSOR_NOT_INSTALLED;
    mag_data_.status = SENSOR_NOT_INSTALLED;
    return false;
  } else {
    data_.status = SENSOR_INSTALLED;
    mag_data_.status = SENSOR_INSTALLED;
  }
  data_.status = SENSOR_INITIALIZING;
  mag_data_.status = SENSOR_INITIALIZING;
  if (!imu_.ConfigAccelRange(config_.accel_range_g)) {
    data_.status = SENSOR_FAULT;
    mag_data_.status = SENSOR_FAULT;
    return false;
  }
  if (!imu_.ConfigGyroRange(config_.gyro_range_dps)) {
    data_.status = SENSOR_FAULT;
    mag_data_.status = SENSOR_FAULT;
    return false;
  }
  if (!imu_.ConfigDlpfBandwidth(config_.dlpf_hz)) {
    data_.status = SENSOR_FAULT;
    mag_data_.status = SENSOR_FAULT;
    return false;
  }
  if (!imu_.ConfigSrd(static_cast<uint8_t>(config_.sample_rate))) {
    data_.status = SENSOR_FAULT;
    mag_data_.status = SENSOR_FAULT;
    return false;
  }
  if (!imu_.EnableDrdyInt()) {
    data_.status = SENSOR_FAULT;
    mag_data_.status = SENSOR_FAULT;
    return false;
  }
  /* Compute the frequency and period */
  freq_hz_ = 1000.0f /
             static_cast<float>(static_cast<uint8_t>(config_.sample_rate) + 1);
  period_ms_ = 1.0f / freq_hz_ * 1000.0f;
  if (config_.sample_rate == SAMPLE_RATE_50HZ) {
    mag_period_ms_ = 125.0f;
  } else {
    mag_period_ms_ = 10.0f;
  }
  /* Convert the config to Eigen for ease of use */
  accel_bias_mps2_ = ArrayToEigen(config_.accel_bias_mps2);
  accel_scale_ = ArrayToEigen(config_.accel_scale);
  rotation_ = ArrayToEigen(config_.rotation);
  data_.status = SENSOR_INITIALIZED;
  mag_data_.status = SENSOR_INITIALIZED;
  return true;
}

bool BfsMpu9250::Calibrate() {
  /* Reset the timer the first time this is called */
  if (!latch_) {
    time_ms_ = 0;
    latch_ = true;
  }
  /* Collect data and estimate biases */
  if (time_ms_ < config_.init_time_ms) {
    data_.status = SENSOR_CALIBRATING;
    if (imu_.Read()) {
      gyro_radps_[0] = imu_.gyro_x_radps();
      gyro_radps_[1] = imu_.gyro_y_radps();
      gyro_radps_[2] = imu_.gyro_z_radps();
      gyro_radps_ = rotation_ * gyro_radps_;
      gx_.Update(gyro_radps_[0]);
      gy_.Update(gyro_radps_[1]);
      gz_.Update(gyro_radps_[2]);
    }
  } else {
    gyro_bias_radps_[0] = -gx_.mean();
    gyro_bias_radps_[1] = -gy_.mean();
    gyro_bias_radps_[2] = -gz_.mean();
    data_.status = SENSOR_CALIBRATED;
    return true;
  }
  return false;
}

void BfsMpu9250::ResetCal() {
  latch_ = false;
  gx_.Clear();
  gy_.Clear();
  gz_.Clear();
  gyro_bias_radps_ = {0, 0, 0};
}

bool BfsMpu9250::Read() {
  data_.new_data = imu_.Read();
  mag_data_.new_data = false;
  if (data_.new_data) {
    time_ms_ = 0;
    data_.status = SENSOR_HEALTHY;
    /* Convert IMU data to eigen vectors */
    gyro_radps_[0] = imu_.gyro_x_radps();
    gyro_radps_[1] = imu_.gyro_y_radps();
    gyro_radps_[2] = imu_.gyro_z_radps();
    accel_mps2_[0] = imu_.accel_x_mps2();
    accel_mps2_[1] = imu_.accel_y_mps2();
    accel_mps2_[2] = imu_.accel_z_mps2();
    /* Apply rotation, scale corrections, and biases */
    accel_mps2_ = accel_scale_ * rotation_ * accel_mps2_ + accel_bias_mps2_;
    gyro_radps_ = rotation_ * gyro_radps_ + gyro_bias_radps_;
    /* Convert results to data struct */
    EigenToArray(accel_mps2_, data_.accel_mps2);
    EigenToArray(gyro_radps_, data_.gyro_radps);
    /* Die temperature */
    data_.die_temp_c = imu_.die_temp_c();
    /* Check for new mag data */
    mag_data_.new_data = imu_.new_mag_data();
    if (mag_data_.new_data) {
      mag_time_ms_ = 0;
      mag_data_.status = SENSOR_HEALTHY;
      mag_ut_[0] = imu_.mag_x_ut();
      mag_ut_[1] = imu_.mag_y_ut();
      mag_ut_[2] = imu_.mag_z_ut();
      mag_ut_ = mag_scale_ * rotation_ * mag_ut_ + mag_bias_ut_;
      EigenToArray(mag_ut_, mag_data_.mag_ut);
      mag_data_.die_temp_c = imu_.die_temp_c();
    } else {
      if (mag_time_ms_ > HEALHTY_MULT_ * mag_period_ms_) {
        mag_data_.status = SENSOR_FAULT;
      } else if (mag_time_ms_ > mag_period_ms_) {
        mag_data_.status = SENSOR_MISSED_FRAME;
      }
    }
  } else {
    if (time_ms_ > HEALHTY_MULT_ * period_ms_) {
      data_.status = SENSOR_FAULT;
    } else if (time_ms_ > period_ms_) {
      data_.status = SENSOR_MISSED_FRAME;
    }
  }
  return data_.new_data;
}

}  // namespace bfs
