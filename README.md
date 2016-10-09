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

### SPI Object Declaratioon

**MPU9250(uint8_t csPin)**
An MPU9250 object should be declared, specifying the Teensy chip select pin used. Multiple MPU-9250 or other SPI objects could be used on the same SPI bus, each with their own chip select pin. For example, the following code declares an MPU9250 object called *IMU* with an MPU-9250 sensor located on chip select pin 10.

```C++
MPU9250 IMU(10);
```

### Common Setup Functions
The following two functions are used to setup the MPU-9250 sensor. These should be called once before data collection, typically this is done in the Arduino *void setup()* function. The *begin* function should always be used. Optionally, the *setFilt* function can be used, following *begin*, to set the programmable Digital Low Pass Filter (DLPF) bandwidth, data output rate, and interrupt.

**int begin(String accelRange, String gyroRange)**
This should be called in your setup function, specifying the accelerometer and gyro ranges. It initializes communication with the MPU-9250 and sets up the sensor for reading data. This function returns 0 on a successful initialization and returns -1 on an unsuccesful initialization. If unsuccessful, please check your wiring or try resetting power to the sensor. The following is an example of setting up the MPU-9250, selecting an accelerometer full scale range of +/- 8g and a gyroscope full scale range of +/- 250 degrees per second.

```C++
int beginStatus;
beginStatus = IMU.begin("8G","250DPS");
```

**(optional) int setFilt(String bandwidth, uint8_t SRD)**
This is an optional function to set the programmable Digital Low Pass Filter (DLPF) bandwidth, data output rate, and interrupt. 

By default, if this function is not called:

* No digital filtering is used.
* The gyroscope and temperature sensor are sampled at a rate of 32 kHz, the accelerometer at a rate of 4 kHz, and the magnetometer at a rate of 100 Hz. 
* When data collection functions are called, the most recent data sample is returned. 

This function enables setting the programmable Digital Low Pass Filter (DLPF) bandwidth, data output rate, and interrupt. The following DLPF bandwidths are supported:

| String bandwidth | DLPF Bandwidth | Gyroscope Delay | Accelerometer Delay | Temperature Bandwidth | Temperature Delay |
| --- | --- | --- | --- | --- | --- |
| "184HZ" | 184 Hz | 2.9 ms   | 5.8 ms   | 188 Hz | 1.9 ms  |
| "92HZ"  | 92 Hz  | 3.9 ms   | 7.8 ms   | 98 Hz  | 2.8 ms  |
| "41HZ"  | 41 Hz  | 5.9 ms   | 11.8 ms  | 42 Hz  | 4.8 ms  |
| "20HZ"  | 20 Hz  | 9.9 ms   | 19.8 ms  | 20 Hz  | 8.3 ms  |
| "10HZ"  | 10 Hz  | 17.85 ms | 35.7 ms  | 10 Hz  | 13.4 ms |
| "5HZ"   | 5 Hz   | 33.48 ms | 66.96 ms | 5 Hz   | 18.6 ms |

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
setFiltStatus = IMU.setFilt("41HZ",9);
```

### Common Data Collection Functions
The functions below are used to collect data from the MPU-9250 sensor. Data is returned scaled to engineering units and transformed to a [common axis system](#sensor-orientation). Accelerometer data is returned in units of meters per second per second (m/s/s), gyroscope data in units of radians per second (rad/s), magnetometer data in units of microtesla (uT) and temperature data in degrees Celsius (C). All of the data returned by the function were collected from the MPU-9250 at the same time, so it is preferable to use the function which returns all of the desired data rather than two separate function calls in order to eliminate potential time skews in your results. For example, it would be preferable to use *getMotion6* to get both gyroscope and accelerometer data rather than call *getAccel* followed by *getGyro*. This preference is because the gyroscope and accelerometer data returned by *getMotion6* were all sampled simultaneously whereas using *getAccel* followed by *getGyro* could possibly introduce a time skew between the accelerometer and gyroscope data.

**void getAccel(float* ax, float* ay, float* az)**
*getAccel(float&ast; ax, float&ast; ay, float&ast; az)* samples the MPU-9250 sensor and returns the three-axis accelerometer data as floats in m/s/s.

```C++
float ax, ay, az;
IMU.getAccel(&ax, &ay, &az);
```

**void getGyro(float* gx, float* gy, float* gz)**
*getGyro(float&ast; gx, float&ast; gy, float&ast; gz)* samples the MPU-9250 sensor and returns the three-axis gyroscope data as floats in rad/s.

```C++
float gx, gy, gz;
IMU.getGyro(&gx, &gy, &gz);
```

**void getMag(float* hx, float* hy, float* hz)**
*getMag(float&ast; hx, float&ast; hy, float&ast; hz)* samples the MPU-9250 sensor and returns the three-axis magnetometer data as floats in uT.

```C++
float hx, hy, hz;
IMU.getMag(&hx, &hy, &hz);
```

**void getTemp(float *t)**
*getTemp(float&ast; t)* samples the MPU-9250 sensor and returns the die temperature as a float in C.

```C++
float t;
IMU.getTemp(&t);
```

**void getMotion6(float* ax, float* ay, float* az, float* gx, float* gy, float* gz)**
*getMotion6(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz)* samples the MPU-9250 sensor and returns the three-axis accelerometer data as floats in m/s/s and the three-axis gyroscope data as floats in rad/s.

```C++
float ax, ay, az, gx, gy, gz;
IMU.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
```

**void getMotion7(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* t)**
*getMotion7(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; t)* samples the MPU-9250 sensor and returns the three-axis accelerometer data as floats in m/s/s, the three-axis gyroscope data as floats in rad/s, and the die temperature as a float in C.

```C++
float ax, ay, az, gx, gy, gz, t;
IMU.getMotion7(&ax, &ay, &az, &gx, &gy, &gz, &t);
```

**getMotion9(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz)**
*getMotion9(float&ast; ax, float&ast; ay, float&ast; az, float&ast; gx, float&ast; gy, float&ast; gz, float&ast; hx, float&ast; hy, float&ast; hz)* samples the MPU-9250 sensor and returns the three-axis accelerometer data as floats in m/s/s, the three-axis gyroscope data as floats in rad/s, and the three-axis magnetometer data as floats in uT.

```C++
float ax, ay, az, gx, gy, gz, hx, hy, hz;
IMU.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &hx, &hy, &hz);
```

**getMotion10(float* ax, float* ay, float* az, float* gx, float* gy, float* gz, float* hx, float* hy, float* hz, float* t)**
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

The Teensy pinout is:

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

4.7 kOhm resistors should be used as pullups on SDA and SCL, these resistors should pullup with a 3.3V source.

## SPI

The MPU-9250 pins should be connected as:
   * VDD: this should be a 2.4V to 3.6V power source. This can be supplied by the Teensy 3.3V output.
   * GND: ground.
   * VDDI: digital I/O supply voltage. This should be between 1.71V and VDD. This can be supplied by the Teensy 3.3V output.
   * FSYNC: not used, should be grounded.
   * INT: (optional) used for the interrupt output setup in *setFilt*. Connect to Teensy pin inerrupt is attached to.
   * SDA / SDI: connect to Teensy DOUT, pin 11.
   * SCL / SCLK: connect to Teensy CLK, pin 13.
   * AD0 / SDO: connect to Teensy DIN, pin 12.
   * nCS: connect to Teensy chip select pin. Pin 10 was used in the code snippets in this document and the included examples, but any Teensy digital I/O pin can be used. 
   * AUXDA: not used.
   * AUXCL: not used.

Some breakout boards, including the Embedded Masters breakout board, require slight modification to enable SPI. Please refer to your vendor's documentation.

   # Performance
Timing data was collected for the *getMotion10* function on all supported Teensy devices using I2C and SPI. Interrupts, setup with the *setFilt* library function, were used to call the function on *data ready*. This way, timing was considered just to: communicate with the MPU-9250 sensor, collect the data off its registers, parse and scale the data to engineering units, and transform to the common axis system. This test gives some indication of performance for the various communication protocols and Teensy devices.

|             | Teensy 3.0 | Teensy 3.1/3.2 | Teensy 3.5 | Teensy 3.6 | Teensy LC |
| ----------- | ---------- | -------------- | ---------- | ---------- | --------- |
| CPU setting | 96 MHz     | 96 MHz         | 120 MHz    | 180 MHz    | 48 MHz    |
| I2C         | 765 us     | 682 us         | 667 us     | 638 us     | 980 us    |
| SPI         | 96 us      | 52 us          | 22 us      | 19 us      | 205 us    |
