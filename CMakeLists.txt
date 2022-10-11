# v3.14 required for FetchContent_MakeAvailable
cmake_minimum_required(VERSION 3.14)
if (DEFINED MCU)
  include(FetchContent)
  FetchContent_Declare(
    mcu_support
    GIT_REPOSITORY https://github.com/bolderflight/mcu-support.git
    GIT_TAG v1.1.0
  )
  FetchContent_MakeAvailable(mcu_support)
  # Setting up the toolchain
  set(CMAKE_TOOLCHAIN_FILE "${mcu_support_SOURCE_DIR}/cmake/cortex.cmake")
  # Project information
  project(InvensenseImu
    VERSION 6.0.3
    DESCRIPTION "Invensense IMU sensor driver"
    LANGUAGES CXX
  )
  # Grab the processor and set up definitions and compile options
  include(${mcu_support_SOURCE_DIR}/cmake/config_mcu.cmake)
  configMcu(${MCU} ${mcu_support_SOURCE_DIR})
  # Fetch core
  FetchContent_Declare(
    core
    GIT_REPOSITORY https://github.com/bolderflight/core.git
    GIT_TAG v3.1.3
  )
  FetchContent_MakeAvailable(core)
  # Add the library target
  add_library(invensense_imu
    src/invensense_imu.cpp
    src/invensense_imu.h
    src/mpu9250.cpp
    src/mpu9250.h
    src/mpu6500.cpp
    src/mpu6500.h
  )
  # Link libraries
  target_link_libraries(invensense_imu
    PUBLIC
      core
  )
  # Setup include directories 
  target_include_directories(invensense_imu PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )
  # Example and test if this project is built separately
  if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
    ### MPU-6500

    # Add the spi example target
    add_executable(mpu6500_spi_example examples/cmake/mpu6500/spi.cc)
    # Add the includes
    target_include_directories(mpu6500_spi_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(mpu6500_spi_example
      PRIVATE
        invensense_imu
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(mpu6500_spi_example ${MCU} ${mcu_support_SOURCE_DIR})

    # Add the i2c example target
    add_executable(mpu6500_i2c_example examples/cmake/mpu6500/i2c.cc)
    # Add the includes
    target_include_directories(mpu6500_i2c_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(mpu6500_i2c_example
      PRIVATE
        invensense_imu
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(mpu6500_i2c_example ${MCU} ${mcu_support_SOURCE_DIR})

    # Add the drdy interrupt example
    add_executable(mpu6500_drdy_spi_example examples/cmake/mpu6500/drdy_spi.cc)
    # Add the includes
    target_include_directories(mpu6500_drdy_spi_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(mpu6500_drdy_spi_example
      PRIVATE
        invensense_imu
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(mpu6500_drdy_spi_example ${MCU} ${mcu_support_SOURCE_DIR})

    ### MPU-9250

    # Add the spi example target
    add_executable(mpu9250_spi_example examples/cmake/mpu9250/spi.cc)
    # Add the includes
    target_include_directories(mpu9250_spi_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(mpu9250_spi_example
      PRIVATE
        invensense_imu
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(mpu9250_spi_example ${MCU} ${mcu_support_SOURCE_DIR})

    # Add the i2c example target
    add_executable(mpu9250_i2c_example examples/cmake/mpu9250/i2c.cc)
    # Add the includes
    target_include_directories(mpu9250_i2c_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(mpu9250_i2c_example
      PRIVATE
        invensense_imu
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(mpu9250_i2c_example ${MCU} ${mcu_support_SOURCE_DIR})

    # Add the drdy interrupt example
    add_executable(mpu9250_drdy_spi_example examples/cmake/mpu9250/drdy_spi.cc)
    # Add the includes
    target_include_directories(mpu9250_drdy_spi_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(mpu9250_drdy_spi_example
      PRIVATE
        invensense_imu
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(mpu9250_drdy_spi_example ${MCU} ${mcu_support_SOURCE_DIR})

    # Add the wom example
    add_executable(mpu9250_wom_example examples/cmake/mpu9250/wom_i2c.cc)
    # Add the includes
    target_include_directories(mpu9250_wom_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(mpu9250_wom_example
      PRIVATE
        invensense_imu
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(mpu9250_wom_example ${MCU} ${mcu_support_SOURCE_DIR})
  endif()
endif()
