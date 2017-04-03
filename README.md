# MPU9250
Library for communicating with the [MPU-9250](https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/) nine-axis Inertial Measurement Unit (IMU) using Teensy 3.x and Teensy LC devices.

# Description
The InvenSense MPU-9250 is a System in Package (SiP) that combines two chips: the MPU-6500 three-axis gyroscope and three-axis accelerometer; and the AK8963 three-axis magnetometer. The MPU-9250 supports I2C, up to 400 kHz, and SPI communication, up to 1 MHz for register setup and 20 MHz for data reading. The following selectable full scale sensor ranges are available:

| Gyroscope Full Scale Range | Accelerometer Full Scale Range | Magnetometer Full Scale Range |
| --- | --- | ---  |
| +/- 250 (deg/s)  | +/- 2 (g)  | +/- 4800 (uT) |
| +/- 500 (deg/s)  | +/- 4 (g)  | |
| +/- 1000 (deg/s) | +/- 8 (g)  | |
| +/- 2000 (deg/s) | +/- 16 (g) | |

The MPU-9250 samples the gyroscopes, accelerometers, and magnetometers with 16 bit analog to digital converters. It also features programmable digital filters, a precision clock, an embedded temperature sensor, and programmable interrupts.

# Usage
This library supports both I2C and SPI commmunication with the MPU-9250. The [i2c_t3 enhanced I2C library](https://github.com/nox771/i2c_t3) for Teensy 3.x/LC devices is used for I2C communication.

## Installation
Simply clone or download this library into your Arduino/libraries folder. The [i2c_t3 enhanced I2C library](https://github.com/nox771/i2c_t3) is bundled with the [Teensyduino software](http://pjrc.com/teensy/td_download.html) and is not required to download separately.

## Function Description
This library supports both I2C and SPI communication with the MPU-9250. The *MPU9250* object declaration is overloaded with different declarations for I2C and SPI communication. All other functions remain the same. 

### I2C Object Declaration

**MPU9250(uint8_t address, uint8_t bus)**
An MPU9250 object should be declared, specifying the MPU-9250 I2C address and the Teensy I2C bus number. The MPU-9250 I2C address will be 0x68 if the AD0 pin is grounded or 0x69 if the AD0 pin is pulled high. For example, the following code declares an MPU9250 object called *IMU* with an MPU-9250 sensor located on Teensy I2C bus 0 (pins 18 and 19) with a sensor address of 0x68 (AD0 grounded).

```C++
MPU9250 IMU(0x68, 0);
```

**MPU9250(uint8_t address, uint8_t bus, i2c_pins pins)**
Optionally, the I2C pins can be specified, which is useful for accessing the [alternate I2C pins](https://github.com/nox771/i2c_t3/#pins) for a given bus. If these aren't specified, this library uses the default pins for a given I2C bus. In this case, an MPU9250 object should be declared, specifying the MPU-9250 I2C address, the Teensy I2C bus number, and the I2C pins. The MPU-9250 I2C address will be 0x68 if the AD0 pin is grounded or 0x69 if the AD0 pin is pulled high. For example, the following code declares an MPU9250 object called *IMU* with an MPU-9250 sensor located on Teensy I2C bus 0, alternate I2C bus 0 pins 16 and 17, with a sensor address of 0x68 (AD0 grounded).

```C++
MPU9250 IMU(0x68, 0, I2C_PINS_16_17);
```

**MPU9250(uint8_t address, uint8_t bus, i2c_pins pins, i2c_pullup pullups)**
Optionally, the I2C pins and pullups can be specified. This is useful for accessing the [alternate I2C pins](https://github.com/nox771/i2c_t3/#pins) for a given bus and using the Teensy's [internal pullups](https://github.com/nox771/i2c_t3/#pullups). If these aren't specified, this library uses the default pins for a given I2C bus and external pullups. In this case, an MPU9250 object should be declared, specifying the MPU-9250 I2C address, the Teensy I2C bus number, the I2C pins, and the I2C pullups. The MPU-9250 I2C address will be 0x68 if the AD0 pin is grounded or 0x69 if the AD0 pin is pulled high. For example, the following code declares an MPU9250 object called *IMU* with an MPU-9250 sensor located on Teensy I2C bus 0, alternate I2C bus 0 pins 16 and 17,  with internal pullups and a sensor address of 0x68 (AD0 grounded).

```C++
MPU9250 IMU(0x68, 0, I2C_PINS_16_17, I2C_PULLUP_INT);
```

### SPI Object Declaratioon

**MPU9250(uint8_t csPin)**
An MPU9250 object should be declared, specifying the Teensy chip select pin used. Multiple MPU-9250 or other SPI objects could be used on the same SPI bus, each with their own chip select pin. SPI Bus 0 is used with the default MOSI, MISO, and SCK pins. The chip select pin can be any available digital pin. For example, the following code declares an MPU9250 object called *IMU* with an MPU-9250 sensor located on chip select pin 10.

```C++
MPU9250 IMU(10);
```

**MPU9250(uint8_t csPin, spi_mosi_pin pin)**
Optionally, the SPI MOSI pin can be specified. This allows selecting SPI buses other than SPI Bus 0 and pins other than the defaults. The enumerated pin names and assigned pin numbers are:

*Teensy 3.0, 3.1, and 3.2*

| MOSI Pin Name | MOSI Pin Number | MISO Pin Number | SCK Pin Number |
| ------------- | --------------- | --------------- | -------------- |
| MOSI_PIN_7    | 7               | 8               | 14             |
| MOSI_PIN_11   | 11              | 12              | 13             |

*Teensy 3.5 and 3.6*

| MOSI Pin Name | MOSI Pin Number | MISO Pin Number | SCK Pin Number |
| ------------- | --------------- | --------------- | -------------- |
| MOSI_PIN_0    | 0               | 1               | 32             |
| MOSI_PIN_7    | 7               | 8               | 14             |
| MOSI_PIN_11   | 11              | 12              | 13             |
| MOSI_PIN_21   | 21              | 5               | 20             |
| MOSI_PIN_28   | 28              | 39              | 27             |
| MOSI_PIN_44   | 44              | 45              | 46             |
| MOSI_PIN_52   | 52              | 51              | 53             |


*Teensy LC*

| MOSI Pin Name | MOSI Pin Number | MISO Pin Number | SCK Pin Number |
| ------------- | --------------- | --------------- | -------------- |
| MOSI_PIN_0    | 0               | 1               | 20             |
| MOSI_PIN_7    | 7               | 8               | 14             |
| MOSI_PIN_11   | 11              | 12              | 13             |
| MOSI_PIN_21   | 21              | 5               | 20             |

For example, the following code declares an MPU9250 object called *IMU* with an MPU-9250 sensor located on chip select pin 10 and MOSI pin 7.

```C++
MPU9250 IMU(10, MOSI_PIN_7);
```

### Common Setup Functions
The following functions are used to setup the MPU-9250 sensor. These should be called once before data collection, typically this is done in the Arduino *void setup()* function. The *begin* function should always be used. Optionally, the *setFilt* function can be used, following *begin*, to set the programmable Digital Low Pass Filter (DLPF) bandwidth, data output rate, and interrupt. Optionally, the *enableInt* function can be used to enable or disable the interrupt generated by the MPU-9250 without setting the programmable Digital Low Pass Filter (DLPF).

**int begin(mpu9250_accel_range accelRange, mpu9250_gyro_range gyroRange)**
This should be called in your setup function, specifying the accelerometer and gyro ranges. It initializes communication with the MPU-9250 and sets up the sensor for reading data. This function returns 0 on a successful initialization and returns -1 on an unsuccesful initialization. If unsuccessful, please check your wiring or try resetting power to the sensor. The following is an example of setting up the MPU-9250, selecting an accelerometer full scale range of +/- 8g and a gyroscope full scale range of +/- 250 degrees per second.

```C++
int beginStatus;
beginStatus = IMU.begin(ACCEL_RANGE_8G,GYRO_RANGE_250DPS);
```

The enumerated accelerometer and gyroscope ranges are:

| Accelerometer Name | Accelerometer Full Scale Range | Gyroscope Name     | Gyroscope Full Scale Range |
| ------------------ | ------------------------------ | ------------------ | -------------------------- |
| ACCEL_RANGE_2G     | +/- 2 (g)                      | GYRO_RANGE_250DPS  | +/- 250 (deg/s)            |
| ACCEL_RANGE_4G     | +/- 4 (g)                      | GYRO_RANGE_500DPS  | +/- 500 (deg/s)            |
| ACCEL_RANGE_8G     | +/- 8 (g)                      | GYRO_RANGE_1000DPS | +/- 1000 (deg/s)           |
| ACCEL_RANGE_16G    | +/- 16 (g)                     | GYRO_RANGE_2000DPS | +/- 2000 (deg/s)           |

**(optional) int setFilt(mpu9250_dlpf_bandwidth bandwidth, uint8_t SRD)**
This is an optional function to set the programmable Digital Low Pass Filter (DLPF) bandwidth, data output rate, and interrupt. 

By default, if this function is not called:

* No digital filtering is used.
* The gyroscope and temperature sensor are sampled at a rate of 32 kHz, the accelerometer at a rate of 4 kHz, and the magnetometer at a rate of 100 Hz. 
* When data collection functions are called, the most recent data sample is returned. 

This function enables setting the programmable Digital Low Pass Filter (DLPF) bandwidth, data output rate, and interrupt. The following DLPF bandwidths are supported:

| Bandwidth Name | DLPF Bandwidth | Gyroscope Delay | Accelerometer Delay | Temperature Bandwidth | Temperature Delay |
| --- | --- | --- | --- | --- | --- |
| DLPF_BANDWIDTH_184HZ | 184 Hz | 2.9 ms   | 5.8 ms   | 188 Hz | 1.9 ms  |
| DLPF_BANDWIDTH_92HZ  | 92 Hz  | 3.9 ms   | 7.8 ms   | 98 Hz  | 2.8 ms  |
| DLPF_BANDWIDTH_41HZ  | 41 Hz  | 5.9 ms   | 11.8 ms  | 42 Hz  | 4.8 ms  |
| DLPF_BANDWIDTH_20HZ  | 20 Hz  | 9.9 ms   | 19.8 ms  | 20 Hz  | 8.3 ms  |
| DLPF_BANDWIDTH_10HZ  | 10 Hz  | 17.85 ms | 35.7 ms  | 10 Hz  | 13.4 ms |
| DLPF_BANDWIDTH_5HZ   | 5 Hz   | 33.48 ms | 66.96 ms | 5 Hz   | 18.6 ms |

The data output rate is set by a sample rate divider, *uint8_t SRD*. The data output rate is then given by:

*Data Output Rate = 1000 / (1 + SRD)*

This allows the data output rate for the gyroscopes, accelerometers, and temperature sensor to be set between 3.9 Hz and 1000 Hz. Note that data should be read at or above the selected rate. In order to prevent aliasing, the data should be sampled at twice the frequency of the bandwidth or higher. For example, this means for a DLPF bandwidth set to 41 Hz, the data output rate and data collection should be at frequencies of 82 Hz or higher.

The magnetometer is fixed to an output rate of: 
* 100 Hz for frequencies of 100 Hz or above (SRD less than or equal to 9)
* 8 Hz for frequencies below 100 Hz (SRD greater than 9)

When the data is read above the selected output rate, the read data will be stagnant. For example, when the output rate is selected to 1000 Hz, the magnetometer data will be the same for 10 sequential frames. 

An interrupt is tied to the data output rate. The MPU-9250 *INT* pin will issue a 50us pulse when data is ready. This is extremely useful for using interrupts to clock data collection that should occur at a regular interval. Please see the *Interrupt_I2C* and *Interrupt_SPI examples*.

Below is an example of selecting a 41 Hz DLPF bandwidth and a data output rate of 100 Hz. This function returns 0 on success and -1 on failure.

```C++
int setFiltStatus;
setFiltStatus = IMU.setFilt(DLPF_BANDWIDTH_41HZ,9);
```

**(optional) int enableInt(bool enable)**
This is an optional function to enable or disable the MPU-9250 generated interrupt without setting the programmable Digital Low Pass Filters (DLPF). When enabled a 50us pulse is issued when data is ready. When the programmable Digital Low Pass Filters (DLPF) are not used, this is typically at the 4 kHz accelerometer data rate. *true* is passed to the function to enable the interrupt and *false* is passed to disable the interrupt. Below is an example of enabling the interrupt.

```C++
int intStatus;
intStatus = IMU.enableInt(true);
```

### Common Data Collection Functions
The functions below are used to collect data from the MPU-9250 sensor. Data is returned scaled to engineering units and transformed to a [common axis system](#sensor-orientation). Accelerometer data is returned in units of meters per second per second (m/s/s), gyroscope data in units of radians per second (rad/s), magnetometer data in units of microtesla (uT) and temperature data in degrees Celsius (C). All of the data returned by the function were collected from the MPU-9250 at the same time, so it is preferable to use the function which returns all of the desired data rather than two separate function calls in order to eliminate potential time skews in your results. For example, it would be preferable to use *getMotion6* to get both gyroscope and accelerometer data rather than call *getAccel* followed by *getGyro*. This preference is because the gyroscope and accelerometer data returned by *getMotion6* were all sampled simultaneously whereas using *getAccel* followed by *getGyro* could possibly introduce a time skew between the accelerometer and gyroscope data.

In addition to the functions below, any of the functions followed by *Counts* will return the data as int16_t counts rather than floats scaled to engineering units. These counts are still transformed to the [common axis system](#sensor-orientation). For example, the following returns the accelerometer data as int16_t counts, transformed to the common axis system:

 ```C++
int16_t ax, ay, az;
IMU.getAccelCounts(&ax, &ay, &az);
```

**void getAccel(float&ast; ax, float&ast; ay, float&ast; az)**
*getAccel(float&ast; ax, float&ast; ay, float&ast; az)* samples the MPU-9250 sensor and returns the three-axis accelerometer data as floats in m/s/s.

```C++
float ax, ay, az;
IMU.getAccel(&ax, &ay, &az);
```

**void getGyro(float&ast; gx, float&ast; gy, float&ast; gz)**
*getGyro(float&ast; gx, float&ast; gy, float&ast; gz)* samples the MPU-9250 sensor and returns the three-axis gyroscope data as floats in rad/s.

```C++
float gx, gy, gz;
IMU.getGyro(&gx, &gy, &gz);
```

**void getMag(float&ast; hx, float&ast; hy, float&ast; hz)**
*getMag(float&ast; hx, float&ast; hy, float&ast; hz)* samples the MPU-9250 sensor and returns the three-axis magnetometer data as floats in uT.

```C++
float hx, hy, hz;
IMU.getMag(&hx, &hy, &hz);
```

**void getTemp(float&ast; t)**
*getTemp(float&ast; t)* samples the MPU-9250 sensor and returns the die temperature as a float in C.

```C++
float t;
IMU.getTemp(&t);
```

**void getMotion6(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz)**
*getMotion6(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz)* samples the MPU-9250 sensor and returns the three-axis accelerometer data as floats in m/s/s and the three-axis gyroscope data as floats in rad/s.

```C++
float ax, ay, az, gx, gy, gz;
IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
```

**void getMotion7(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; t)**
*getMotion7(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; t)* samples the MPU-9250 sensor and returns the three-axis accelerometer data as floats in m/s/s, the three-axis gyroscope data as floats in rad/s, and the die temperature as a float in C.

```C++
float ax, ay, az, gx, gy, gz, t;
IMU.getMotion7(&ax, &ay, &az, &gx, &gy, &gz, &t);
```

**getMotion9(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; hx, float&ast; hy, float&ast; hz)**
*getMotion9(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; hx, float&ast; hy, float&ast; hz)* samples the MPU-9250 sensor and returns the three-axis accelerometer data as floats in m/s/s, the three-axis gyroscope data as floats in rad/s, and the three-axis magnetometer data as floats in uT.

```C++
float ax, ay, az, gx, gy, gz, hx, hy, hz;
IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
```

**getMotion10(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; hx, float&ast; hy, float&ast; hz, float&ast; t)**
*getMotion10(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; hx, float&ast; hy, float&ast; hz, float&ast; t)* samples the MPU-9250 sensor and returns the three-axis accelerometer data as floats in m/s/s, the three-axis gyroscope data as floats in rad/s, the three-axis magnetometer data as floats in uT, and the die temperature as a float in C.

```C++
float ax, ay, az, gx, gy, gz, hx, hy, hz, t;
IMU.getMotion10(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz, &t);
```

##<a name="sensor-orientation"></a>Sensor Orientation
This library transforms all data to a common axis system before it is returned. This axis system is shown below. It is a right handed coordinate system with the z-axis positive down, common in aircraft dynamics.

<img src="https://github.com/bolderflight/MPU9250/blob/master/docs/MPU-9250-AXIS.png" alt="Common Axis System" width="250">

**Caution!** This axis system is shown relative to the MPU-9250 sensor. The sensor may be rotated relative to the breakout board. 

## Example List

* **Basic_I2C**: demonstrates declaring an *MPU9250* object, initializing the sensor, and collecting data. I2C is used to communicate with the MPU-9250 sensor.
* **Basic_I2C_Pins**: demonstrates declaring an *MPU9250* object, initializing the sensor, and collecting data. I2C is used to communicate with the MPU-9250 sensor and the I2C pins are specified. In this example, I2C bus 0 alternate pins 16 and 17 are demonstrated.
* **Basic_I2C_Pullups**: demonstrates declaring an *MPU9250* object, initializing the sensor, and collecting data. I2C is used to communicate with the MPU-9250 sensor and the I2C pins and pullups are specified. In this example, I2C bus 0 alternate pins 16 and 17 are demonstrated with internal pullups.
* **Basic_SPI**: demonstrates declaring an *MPU9250* object, initializing the sensor, and collecting data. SPI is used to communicate with the MPU-9250 sensor.
* **Interrupt_I2C**: demonstrates a more advanced setup. In this case an *MPU9250* object is declared, the sensor is initialized and the filter and interrupt are setup. The MPU-9250 sensor creates an interrupt pulse when data is ready, which is used to drive data collection at the specified rate. I2C is used to communicate with the MPU-9250 sensor.
* **Interrupt_SPI**: demonstrates a more advanced setup. In this case an *MPU9250* object is declared, the sensor is initialized and the filter and interrupt are setup. The MPU-9250 sensor creates an interrupt pulse when data is ready, which is used to drive data collection at the specified rate. SPI is used to communicate with the MPU-9250 sensor.

# Wiring and Pullups 

Please refer to the [MPU-9250 datasheet](https://github.com/bolderflight/MPU9250/blob/master/docs/MPU-9250-Datasheet.pdf) and the [Teensy pinout diagrams](https://www.pjrc.com/teensy/pinout.html). This library was developed using the [Embedded Masters breakout board](https://store.invensense.com/Controls/www.embeddedmasters.com/ProductDetail/EMSENSRMPU9250-Embedded-Masters/552444/) v1.1 for the MPU-9250. The data sheet for this breakout board is located [here](https://github.com/bolderflight/MPU9250/blob/master/docs/Embedded-Masters-MPU-9250-Breakout.pdf). This library should work well for other breakout boards or embedded sensors, please refer to your vendor's pinout diagram.

## I2C

The MPU-9250 pins should be connected as:
   * VDD: this should be a 2.4V to 3.6V power source. This can be supplied by the Teensy 3.3V output.
   * GND: ground.
   * VDDI: digital I/O supply voltage. This should be between 1.71V and VDD. This can be supplied by the Teensy 3.3V output.
   * FSYNC: not used, should be grounded.
   * INT: (optional) used for the interrupt output setup in *setFilt*. Connect to Teensy pin inerrupt is attached to.
   * SDA / SDI: connect to Teensy SDA.
   * SCL / SCLK: connect to Teensy SCL.
   * AD0 / SDO: ground to select I2C address 0x68. Pull high to VDD to select I2C address 0x69.
   * nCS: no connect.
   * AUXDA: not used.
   * AUXCL: not used.

By default, the Teensy pinout is:

   * Teensy 3.0:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
   * Teensy 3.1/3.2:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 29: SCL, Pin 30: SDA
   * Teensy 3.5:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 37: SCL, Pin 38: SDA
      * Bus 2 - Pin 3: SCL, Pin 4: SDA
   * Teensy 3.6:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 37: SCL, Pin 38: SDA
      * Bus 2 - Pin 3: SCL, Pin 4: SDA
      * Bus 3 - Pin 56: SDA, Pin 57: SCL
   * Teensy LC:
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 22: SCL, Pin 23: SDA  

Alternatively, if the *MPU9250* object is declared specifying the I2C pins used, the Teensy pinout is:

   * Teensy 3.0:
      * Bus 0 - Pin 16: SCL, Pin 17: SDA
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
   * Teensy 3.1/3.2:
      * Bus 0 - Pin 16: SCL, Pin 17: SDA
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 26: SCL, Pin 31: SDA
      * Bus 1 - Pin 29: SCL, Pin 30: SDA
   * Teensy 3.5:
      * Bus 0 - Pin 7: SCL, Pin 8: SDA
      * Bus 0 - Pin 16: SCL, Pin 17: SDA
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 0 - Pin 33: SCL, Pin 34: SDA
      * Bus 0 - Pin 47: SCL, Pin 48: SDA
      * Bus 1 - Pin 37: SCL, Pin 38: SDA
      * Bus 2 - Pin 3: SCL, Pin 4: SDA
   * Teensy 3.6:
      * Bus 0 - Pin 7: SCL, Pin 8: SDA
      * Bus 0 - Pin 16: SCL, Pin 17: SDA
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 0 - Pin 33: SCL, Pin 34: SDA
      * Bus 0 - Pin 47: SCL, Pin 48: SDA
      * Bus 1 - Pin 37: SCL, Pin 38: SDA
      * Bus 2 - Pin 3: SCL, Pin 4: SDA
      * Bus 3 - Pin 56: SDA, Pin 57: SCL
   * Teensy LC:
      * Bus 0 - Pin 16: SCL, Pin 17: SDA
      * Bus 0 - Pin 18: SDA, Pin 19: SCL
      * Bus 1 - Pin 22: SCL, Pin 23: SDA 

4.7 kOhm resistors should be used as pullups on SDA and SCL, these resistors should pullup with a 3.3V source. In some very limited cases with the Teensy 3.0, 3.1/3.2, and 3.5, internal pullups could be used for a single device on a short bus. In this case, the *MPU9250* object should be declared specifying the I2C pins used and the I2C pullup. 

## SPI

The MPU-9250 pins should be connected as:
   * VDD: this should be a 2.4V to 3.6V power source. This can be supplied by the Teensy 3.3V output.
   * GND: ground.
   * VDDI: digital I/O supply voltage. This should be between 1.71V and VDD. This can be supplied by the Teensy 3.3V output.
   * FSYNC: not used, should be grounded.
   * INT: (optional) used for the interrupt output setup in *setFilt*. Connect to Teensy pin inerrupt is attached to.
   * SDA / SDI: connect to Teensy MOSI.
   * SCL / SCLK: connect to Teensy SCK.
   * AD0 / SDO: connect to Teensy MISO.
   * nCS: connect to Teensy chip select pin. Pin 10 was used in the code snippets in this document and the included examples, but any Teensy digital I/O pin can be used. 
   * AUXDA: not used.
   * AUXCL: not used.

By default, the Teensy pinout is:

   * SPI Bus 0 - Pin 11: MOSI, Pin 12: MISO, Pin 13: SCK

Alternatively, if the *MPU9250* object is declared specifying the MOSI pin used, the Teensy pinout is:

   * Teensy 3.0, 3.1, and 3.2:
      * Bus 0 - Pin 7: MOSI, Pin 8: MISO, Pin 14: SCK
      * Bus 0 - Pin 11: MOSI, Pin 12: MISO, Pin 13: SCK
   * Teensy 3.5 and 3.6:
      * Bus 0 - Pin 7: MOSI, Pin 8: MISO, Pin 14: SCK
      * Bus 0 - Pin 11: MOSI, Pin 12: MISO, Pin 13: SCK
      * Bus 0 - Pin 28: MOSI, Pin 39: MISO, Pin 27: SCK
      * Bus 1 - Pin 0: MOSI, Pin 1: MISO, Pin 32: SCK
      * Bus 1 - Pin 21: MOSI, Pin 5: MISO, Pin 20: SCK
      * Bus 2 - Pin 44: MOSI, Pin 45: MISO, Pin 46: SCK
      * Bus 2 - Pin 52: MOSI, Pin 51: MISO, Pin 53: SCK
   * Teensy LC:
      * Bus 0 - Pin 7: MOSI, Pin 8: MISO, Pin 14: SCK
      * Bus 0 - Pin 11: MOSI, Pin 12: MISO, Pin 13: SCK
      * Bus 1 - Pin 0: MOSI, Pin 1: MISO, Pin 20: SCK
      * Bus 1 - Pin 21: MOSI, Pin 5: MISO, Pin 20: SCK

Some breakout boards, including the Embedded Masters breakout board, require slight modification to enable SPI. Please refer to your vendor's documentation.

# Performance
Timing data was collected for the *getMotion10* function on all supported Teensy devices using I2C and SPI. Interrupts, setup with the *setFilt* library function, were used to call the function on *data ready*. This way, timing was considered just to: communicate with the MPU-9250 sensor, collect the data off its registers, parse and scale the data to engineering units, and transform to the common axis system. This test gives some indication of performance for the various communication protocols and Teensy devices.

|             | Teensy 3.0 | Teensy 3.1/3.2 | Teensy 3.5 | Teensy 3.6 | Teensy LC |
| ----------- | ---------- | -------------- | ---------- | ---------- | --------- |
| CPU setting | 96 MHz     | 96 MHz         | 120 MHz    | 180 MHz    | 48 MHz    |
| I2C         | 765 us     | 682 us         | 667 us     | 638 us     | 980 us    |
| SPI         | 96 us      | 52 us          | 22 us      | 19 us      | 205 us    |
