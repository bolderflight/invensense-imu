/*
 * AHRS_SPI.ino
 * Pat Deegan, contact through https://flyingcarsandstuff.com
 * 
 * Copyright (C) 2018 Pat Deegan, psychogenic.com
 * 
 * Basic SPI-version of the MPU test, augmented with 
 *  - optional calibration steps (set by #defines below)
 *  - Madgwick AHRS filtering
 *  - Roll, pitch, yaw estimates output
 *  
 *  Note: depending on your MCU and settings (configuration #defines below), 
 *  it may take a few seconds for the readings to settle and match reality.
 *  
 *  
 *  Secrets of AHRS:
 *  
 *   1) In addition to regular IMU object, create an instance of 
 *      Madgwick myFilter;
 *      
 *   2) During your setup, call
 *      myFilter.begin(SAMPLING_FREQ); (see config section, below for details)
 *   
 *   3) Pass in that filter object in your calls to readSensor(), 
 *      IMU.readSensor(myFilter);
 *     and it will be updated with each read.
 *     
 *   4) Get current AHRS estimates from the filter object using, e.g.
 *      myFilter.getRoll() or myFilter.getRollRadians(), etc
 *     
 *  
 *  
Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, 
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or 
substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *  
 */
#include <SPI.h>
#include <MPU9250.h>


/* ***** Configuration ******
 *  
 * Short version: get as many samples as reasonable but don't lie to your 
 * Madgwick filter... 
 * 
 * Figure out your effective MFILTER_SAMPLE_FREQ_HZ and set 
 * IMU_LOWPASSFILTER_BANDWIDTH accordingly.
 * 
 * For example, on an 80MHz ESP32, setting:
 * 
 *   IMU_POLL_DELAY_MS              2
 *   MFILTER_SAMPLE_FREQ_HZ         480
 *   IMU_LOWPASSFILTER_BANDWIDTH    MPU9250::DLPF_BANDWIDTH_182HZ
 *   
 * worked smashingly, whereas on a 3v3 8MHz Pro Mini:
 *   IMU_POLL_DELAY_MS              5
 *   MFILTER_SAMPLE_FREQ_HZ         100
 *   IMU_LOWPASSFILTER_BANDWIDTH    MPU9250::DLPF_BANDWIDTH_41HZ
 *   
 * does a good job, once its settled.
 *   
 */


// the SPI CS pin you're using
#define CHIP_SELECT_PIN         10


#define SERIAL_BAUD_RATE        57600




// IMU_POLL_DELAY_MS -- delay between loops/readSensor, in ms.
// this is:
//  - the (approx) inverse of your max sampling frequency...
//    so, e.g. for 2ms: freq = 1000/2 = 500Hz
//              for 5ms: freq = 1000/5 = 200Hz
//  - affects your setting for MFILTER_SAMPLE_FREQ_HZ, below
//  - affect your setting for the DLPF, you should poll/readSensor
//    at least twice as often as whatever is set in there
#define IMU_POLL_DELAY_MS         5



// MFILTER_SAMPLE_FREQ_HZ -- tells the Madgwick filter "how
// long each update applies"...
// this should be set according to whatever is delaying your
// reads and updates to the filter.  In this case, it's mostly the
// IMU_POLL_DELAY_MS above, plus a little fudge factor as we're
// spending time doing the reads and the crunching and the reporting...

// this parameter may require some tweaking... start by taking your nominal
// max frequency based on IMU_POLL_DELAY_MS or your specific circumstances
// and 
#define MFILTER_SAMPLE_FREQ_HZ    100


#define IMU_LOWPASSFILTER_BANDWIDTH   MPU9250::DLPF_BANDWIDTH_41HZ




// CALIB_DISABLE -- when defined, calibration values are ignored,
// even if *_CALIB_DONE is defined (below).
#define CALIB_DISABLE

// ACC_CALIB_DONE -- undefine this to force a manual calibration
// of the accelerometer on startup.  You'll have to hold the IMU
// steady in all 6 directions, and when done, take note of the results
// to stick them into the setup()
#define ACC_CALIB_DONE

// MAG_CALIB_DONE -- undefine this to force manual calib for the
// mag sensor on startup.  It's pretty annoying.  You're to gently move
// the IMU around until you've hit max & min on every axis, meaning the
// thing has to eventually point at every direction in the 3-sphere around
// you.
// make note of the output values when it's done, and stick those in the setup()
#define MAG_CALIB_DONE




/* **** Globals ***** */

// an MPU9250 object with the MPU-9250 sensor on SPI bus 0 and chip select pin 10
MPU9250 IMU(SPI, CHIP_SELECT_PIN);

// the Madgwick AHRS filter
Madgwick AHRSFilter;

int status;

// a little container for my bunch of AHRS bytes
typedef union {
  struct {
    float roll;
    float pitch;
    float yaw;
  };
  uint8_t valArray[sizeof(float) * 3];
} NotifBytesBunch;

NotifBytesBunch AHRSValues;

void setup() {
  // serial to display data
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  IMU.setGyroRange(MPU9250::GYRO_RANGE_1000DPS);
  IMU.setDlpfBandwidth(IMU_LOWPASSFILTER_BANDWIDTH);
  
#ifdef ACC_CALIB_DONE

#ifndef CALIB_DISABLE
  // params are the bias/scaleFactor reported by calib step
  IMU.setAccelCalX(-0.015069, 1.001090);
  IMU.setAccelCalY(-0.015069, 1.007469);
  IMU.setAccelCalZ(0, 1);
#endif

#else
  Serial.println(F("********** ACC calib **************"));

  char * dirs[6] = { "X+", "X-", "Y+", "Y-", "Z+", "Z-"};

  for (uint8_t i = 0; i < 6; i++) {
    Serial.print(F("Enter when ready for dir "));
    Serial.print((int)(i + 1));
    Serial.print(' ');
    Serial.print(dirs[i]);
    while (! Serial.available() ) {
      delay(10);
    }

    while (Serial.available()) {
      Serial.read();
      delay(5);
      Serial.print('.');
    }
    Serial.println();
    IMU.calibrateAccel();
  }

  Serial.println(F("Acc calib done"));

  Serial.println(F("Vals: "));

  Serial.print(F("X: "));
  Serial.print(IMU.getAccelBiasX_mss(), 6);
  Serial.print('/');
  Serial.println(IMU.getAccelScaleFactorX(), 6);

  Serial.print(F("Y: "));
  Serial.print(IMU.getAccelBiasY_mss(), 6);
  Serial.print('/');
  Serial.println(IMU.getAccelScaleFactorY(), 6);


  Serial.print(F("Z: "));
  Serial.print(IMU.getAccelBiasZ_mss(), 6);
  Serial.print('/');
  Serial.println(IMU.getAccelScaleFactorZ(), 6);

#endif



#ifdef MAG_CALIB_DONE
 #ifndef CALIB_DISABLE
  // params are the bias/scaleFactor reported by calib step
  IMU.setMagCalX(9.370945, 0.979867);
  IMU.setMagCalY(29.952621, 0.997284);
  IMU.setMagCalZ(-18.193622, 1.023824);
 #endif

#else
  Serial.print(F("CALIB MAG -- move in figure 8s until I say stop!!!"));
  delay(500);
  IMU.calibrateMag();
  Serial.println(F(" done!"));

  Serial.print(F("X: "));
  Serial.print(IMU.getMagBiasX_uT(), 6);
  Serial.print('/');
  Serial.println(IMU.getMagScaleFactorX(), 6);

  Serial.print(F("Y: "));
  Serial.print(IMU.getMagBiasY_uT(), 6);
  Serial.print('/');
  Serial.println(IMU.getMagScaleFactorY(), 6);



  Serial.print(F("Z: "));
  Serial.print(IMU.getMagBiasZ_uT(), 6);
  Serial.print('/');
  Serial.println(IMU.getMagScaleFactorZ(), 6);
#endif

  AHRSFilter.begin(MFILTER_SAMPLE_FREQ_HZ);

}

uint8_t loopCount = 0;

void loop() {
  // read the sensor, passing in the filter 
  // so it will be updated
  IMU.readSensor(AHRSFilter);

  // output periodically
  if (loopCount++ > 10) {
    loopCount = 0;

    float rolly = AHRSFilter.getRoll();
    if (rolly < 0) {
      // I like them positive
      rolly = 360.0 + rolly;
    }

    AHRSValues.roll = rolly;
    AHRSValues.pitch = AHRSFilter.getPitch();
    AHRSValues.yaw = AHRSFilter.getYaw();

    // now you can do something with the AHRSValues

    // we'll just output

    Serial.print("R: ");
    Serial.print(rolly, 2);

    Serial.print("\tP: ");
    Serial.print(AHRSValues.pitch , 2);
    Serial.print("\tY: ");
    Serial.println(AHRSValues.yaw, 2);
  }

  delay(IMU_POLL_DELAY_MS);
}
