## Table of Contents
* [Our Vision](#vision)
* [Getting Started](#getting_started)
   - [Development Environment](#dev_env)
   - [Project Organization](#organization)
   - [Contributing](#contributing)
   - [Style Guide](#style)
   - [Licensing](#licensing)
   - [Examples](#examples)
   - [Tags](#tags)

## Our Vision<a name="vision"></a>
*To drive the rapid growth of reliable autonomous aircraft with a focus on research and commercial applications with the highest potential to improve the environment, improve access to affordable mobility, and expand access to public services and goods. We will drive this rapid growth by developing flight systems with a focus on data quality, reliability, and robustness at a revolutionary price.*

## Getting Started<a name="getting_started"></a>

### Development Environment<a name="dev_env"></a>
We use a Linux based development environment with CMake and GCC as a build system, git for versioning, and Google Protocol Buffers for specifying datalog packets. The following are specific instructions for setting up your development environment.

1. Linux is used as our development environment. Start by installing a Debian-based distro or the Windows Subsystem for Linux (WSL) version 1 or 2. Although other Linux distros should work, these instructions are written for a Debian-based distro.
2. Install *build-essential*

```
$ sudo apt-get install build-essential
```
3. Install OpenSSL development headers

```
$ sudo apt-get install libssl-dev
```
4. Install CMake from source. [Releases](https://github.com/Kitware/CMake/tree/release) and excellent instructions can be found in the [CMake GitHub repo](https://github.com/Kitware/CMake) and summarized below.

```
$ wget https://github.com/Kitware/CMake/releases/download/v3.18.4/cmake-3.18.4.tar.gz
$ tar -xvf cmake-3.18.4.tar.gz
$ cd cmake-3.18.4
$ ./bootstrap
$ make
$ sudo make install
```

5. Install [Google Protocol Buffers](https://developers.google.com/protocol-buffers/). Instructions can be found [here](https://github.com/protocolbuffers/protobuf/blob/master/src/README.md) and summarized below.

```
$ sudo apt-get install autoconf automake libtool curl make g++ unzip
$ wget https://github.com/protocolbuffers/protobuf/releases/download/v3.13.0/protobuf-all-3.13.0.tar.gz
$ tar -xvf protobuf-all-3.13.0.tar.gz
$ cd protobuf-all-3.13.0
$ ./configure
$ make
$ make check
$ sudo make install
$ sudo ldconfig
```

6. Install [Google Protocol Buffers](https://developers.google.com/protocol-buffers/) python language packages. Excellent instructions can be found [here](https://github.com/protocolbuffers/protobuf/tree/master/python) and summarized below. These are required for nanopb to install correctly in the next step.

```
$ sudo apt-get install python3-dev
$ sudo apt-get install python3-setuptools
$ cd protobuf-all-3.13.0/python
$ python3 setup.py build
$ python3 setup.py test
$ python3 setup.py build --cpp_implementation
$ python3 setup.py test --cpp_implementation
$ sudo python3 setup.py install
```

7. Install [nanopb](https://github.com/nanopb/nanopb). This is used for transpiling protocol buffers for the flight computer. Download the latest release and copy it to /usr/local/nanopb.

```
$ wget https://github.com/nanopb/nanopb/archive/0.4.3.tar.gz
$ tar -xvf 0.4.3.tar.gz
$ sudo cp -r nanopb-0.4.3 /usr/local/nanopb
```

8. Install [gcc-arm-none-eabi](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads). This toolchain is used for compiling software for the flight computer. This is done by downloading the latest Linux toolchain and copying the contents to /usr/local.

```
$ wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2020q2/gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz
$ tar -xvf gcc-arm-none-eabi-9-2020-q2-update-x86_64-linux.tar.bz
$ cd gcc-arm-none-eabi-9-2020-q2-update
$ sudo cp -r * /usr/local/
```

9. Install the Linux udev rules. Software is flashed over USB and udev rules are needed to give the loader permission to use the USB port. The rules can be downloaded [here](https://www.pjrc.com/teensy/49-teensy.rules) and are copieied to /etc/udev/rules.d/.

```
$ wget https://www.pjrc.com/teensy/49-teensy.rules
$ sudo cp 49-teensy.rules /etc/udev/rules.d/
```

10. Configure git with your name and email
```
$ git config --global user.name "Your Name"
$ git config --global user.email "youremail@yourdomain.com"
```

11. Install your favorite text editor and start hacking! Hints on installation are given in each repo. In general, the CMake idiom involves creating a *build* directory, running CMake, and running make. For this repo, the steps would be:

```
$ mkdir build
$ cd build
$ cmake ..
$ make
```

This builds the software. To upload to the flight computer, the command would be:

```
$ make flight_upload
```

### Project Organization<a name="organization"></a>
The project is organized into many small repos. This approach enhances the code readability, re-use, and unit testing. CMake's *fetch_content* is used to grab dependencies. A summary of the relevant repos is below:

* [flight](https://gitlab.com/bolderflight/software/flight): flight software template, overarching project vision and direction.
* [core](https://gitlab.com/bolderflight/software/core): microcontroller startup code and Hardware Abstraction Layer (HAL).
* [global_defs](https://gitlab.com/bolderflight/software/global_defs): definitions of constants and unit conversions.
* [eigen](https://gitlab.com/bolderflight/software/eigen): matrix math library.
* [mpu9250](https://gitlab.com/bolderflight/software/mpu9250): driver for the MPU-9250 9 axis IMU.
* [bme280](https://gitlab.com/bolderflight/software/bme280): driver for the BME280 static pressure sensor.
* [ams5812](https://gitlab.com/bolderflight/software/ams5812): driver for the AMS-5812 static and differential pressure transducers.
* [ams5915](https://gitlab.com/bolderflight/software/ams5915): driver for the AMS-5915 static and differential pressure transducers.
* [ublox](https://gitlab.com/bolderflight/software/ublox): driver for uBlox GNSS receivers. Parses the UBX binary format.
* [sbus](https://gitlab.com/bolderflight/software/sbus): driver for reading SBUS inputs from SBUS capable RC receivers and writing SBUS commands to SBUS capable servos.
* [pwm](https://gitlab.com/bolderflight/software/pwm): driver for writing PWM commands to servos.
* [actuator](https://gitlab.com/bolderflight/software/actuator): library for converting actuator commands from angles to SBUS or PWM commands and controlling throttle safety and lost-link actions.
* [sd](https://gitlab.com/bolderflight/software/sd): SD card driver.
* [circle_buf](https://gitlab.com/bolderflight/software/circle_buf): circular buffer with templated type and size.
* [logger](https://gitlab.com/bolderflight/software/logger): generic data logger, buffers and writes data to SD card.
* [checksum](https://gitlab.com/bolderflight/software/checksum): library of checksum algorithms.
* [framing](https://gitlab.com/bolderflight/software/framing): frames data with a start bit, end bit, and checksum. Includes encoder, for writing messages, and a decoder for parsing messages from a data stream.
* [mat_v4](https://gitlab.com/bolderflight/software/mat_v4): functions to write data in MATLAB v4 format.
* [statistics](https://gitlab.com/bolderflight/software/statistics): running estimation of mean, variance, and standard deviation.
* [polytools](https://gitlab.com/bolderflight/software/polytools): polynomial fitting and evaluation.
* [airdata](https://gitlab.com/bolderflight/software/airdata): functions to compute airspeeds and altitudes. Estimations for temperature and density.
* [filter](https://gitlab.com/bolderflight/software/filter): library of digital filters
* [navigation](https://gitlab.com/bolderflight/software/navigation): navigation filters (i.e. EKF to estimate attitude, position, and velocity) and transforms between LLA, ECEF, and NED frames.
* [control](https://gitlab.com/bolderflight/software/control): library of control laws.
* [mavlink_c_library_v1](https://gitlab.com/bolderflight/software/mavlink_c_library_v1): C header files for MAV Link v1.
* [mavlink_c_library_v2](https://gitlab.com/bolderflight/software/mavlink_c_library_v2): C header files for MAV Link v2.
* [mavlink](https://gitlab.com/bolderflight/software/mavlink): wrapper to ease sending MAV Link telemetry.
* [mat_converter](https://gitlab.com/bolderflight/software/mat_converter): program to convert BFS data log files to MATLAB mat files given a Google Protocol Buffer description of the data log message layout.

### Contributing<a name="contributing"></a>
We welcome code contributions, bug reports, suggested enhancements, and assistance with documentation or testing. Use the repo's issue tracking system to identify potential bugs and suggested enhancements. Issue pull requests for making code contributions and assisting with documentation. If you have an idea to contribute and are unsure of the best approach, please contact us at support@bolderflight.com to discuss.

When issuing a bug report, be specific about:
   * The microprocessor used or Linux environment
   * Steps to reproduce the bug
   * Include code

### Style Guide<a name="style"></a>
Follow the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html). Any parameter or variable names, which contain unit specific data should be appended with an underscore and the units (i.e. accel_z_mps2). Units should be named by their common abbreviation. For derived units, use _p_ to indicate _"per"_ and append exponents. So, m/s/s would be _mps2_ and kg/m^3 would be _kgpm3_. If a common abbreviation already exists for the unit, use that instead (i.e. _psi_ instead of _lbpin2_).

Prefer to use _float_ instead of _double_ unless there is a specific and demonstrated need for the additional resolution. Specify integer size in all cases where a certain number of bytes are expected (i.e. uint16_t where 2 bytes is assumed); otherwise use std::size_t or a signed int. Typically, we are not concerned with program size and int work well for integers and index values. If a certain number of bytes are needed they need to be called out directly since different compilers and platforms can change the number of bytes stored in a short, for instance. Do not assume that using unsigned int will prevent a negative input, instead use signed int and check for negative values.

Linting tests check for conformance to the style guide - analyzing the code for potential errors and leading to better readibility. [cpplint](https://raw.githubusercontent.com/google/styleguide/gh-pages/cpplint/cpplint.py) should be used to conduct linting tests with verbosity level 0. Typically line length limits can be ignored for code; although, comments should conform to the line length limitation.

### Licensing<a name="licensing"></a>
If you use external sources, ensure they are licensed MIT, BSD, or a similarly permissive license. We would like to limit the amount of LGPL code and need to avoid GPL and unlicensed code.

If you would like to use external sources with licenses other than MIT or BSD, contact support@bolderflight.com to discuss options.

### Examples<a name="examples"></a>
Develop examples demonstrating your code's functionality and include expected outputs in comments. These examples provide an easy access point to learning your code and ensuring that it installed correctly.

### Tags<a name="tags"></a>
Tags are used to specify release version numbers in [semver](https://semver.org/) formatting. These tags are important because repositories using your code as a dependency will pull, build, and validate against a specific version, rather than continuously needing to manage code updates following HEAD.