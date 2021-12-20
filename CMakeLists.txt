# v3.14 required for FetchContent_MakeAvailable
cmake_minimum_required(VERSION 3.14)
if (DEFINED MCU)
  include(FetchContent)
  FetchContent_Declare(
    mcu_support
    GIT_REPOSITORY https://github.com/bolderflight/mcu-support.git
    GIT_TAG v1.0.0
  )
  FetchContent_MakeAvailable(mcu_support)
  # Setting up the toolchain
  set(CMAKE_TOOLCHAIN_FILE "${mcu_support_SOURCE_DIR}/cmake/cortex.cmake")
  # Project information
  project(Mpu9250
    VERSION 5.4.1
    DESCRIPTION "MPU-9250 sensor driver"
    LANGUAGES CXX
  )
  # Grab the processor and set up definitions and compile options
  include(${mcu_support_SOURCE_DIR}/cmake/config_mcu.cmake)
  configMcu(${MCU} ${mcu_support_SOURCE_DIR})
  # Fetch core
  FetchContent_Declare(
    core
    GIT_REPOSITORY https://github.com/bolderflight/core.git
    GIT_TAG v3.0.2
  )
  FetchContent_MakeAvailable(core)
  # Fetch units
  FetchContent_Declare(
    units
    GIT_REPOSITORY https://github.com/bolderflight/units.git
    GIT_TAG v4.0.4
  )
  FetchContent_MakeAvailable(units)
  # Fetch eigen
  FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://github.com/bolderflight/eigen.git
    GIT_TAG v3.0.2
  )
  FetchContent_MakeAvailable(eigen)
  # Add the library target
  add_library(mpu9250
    src/mpu9250.cpp
    src/mpu9250.h
  )
  # Link libraries
  target_link_libraries(mpu9250
    PUBLIC
      core
      units
      eigen
  )
  # Setup include directories 
  target_include_directories(mpu9250 PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<INSTALL_INTERFACE:include>
  )
  # Example and test if this project is built separately
  if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
    # Add the spi example target
    add_executable(spi_example examples/cmake/spi.cc)
    # Add the includes
    target_include_directories(spi_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(spi_example
      PRIVATE
        mpu9250
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(spi_example ${MCU} ${mcu_support_SOURCE_DIR})

    # Add the i2c example target
    add_executable(i2c_example examples/cmake/i2c.cc)
    # Add the includes
    target_include_directories(i2c_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(i2c_example
      PRIVATE
        mpu9250
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(i2c_example ${MCU} ${mcu_support_SOURCE_DIR})

    # Add the drdy interrupt example
    add_executable(drdy_spi_example examples/cmake/drdy_spi.cc)
    # Add the includes
    target_include_directories(drdy_spi_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(drdy_spi_example
      PRIVATE
        mpu9250
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(drdy_spi_example ${MCU} ${mcu_support_SOURCE_DIR})

    # Add the wom example
    add_executable(wom_example examples/cmake/wom_i2c.cc)
    # Add the includes
    target_include_directories(wom_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(wom_example
      PRIVATE
        mpu9250
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(wom_example ${MCU} ${mcu_support_SOURCE_DIR})

    # Add the fifo example
    add_executable(fifo_example examples/cmake/fifo_spi.cc)
    # Add the includes
    target_include_directories(fifo_example PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(fifo_example
      PRIVATE
        mpu9250
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(fifo_example ${MCU} ${mcu_support_SOURCE_DIR})
  endif()
endif()
