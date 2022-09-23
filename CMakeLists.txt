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
    VERSION 6.0.0
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
    GIT_TAG v3.1.0
  )
  FetchContent_MakeAvailable(core)
  # Fetch eigen
  FetchContent_Declare(
    eigen
    GIT_REPOSITORY https://github.com/bolderflight/eigen.git
    GIT_TAG v3.0.2
  )
  FetchContent_MakeAvailable(eigen)
  # Fetch statistics
  FetchContent_Declare(
    statistics
    GIT_REPOSITORY https://github.com/bolderflight/statistics.git
    GIT_TAG v4.0.4
  )
  FetchContent_MakeAvailable(statistics)
  # Fetch utils
  FetchContent_Declare(
    utils
    GIT_REPOSITORY https://github.com/bolderflight/utils.git
    GIT_TAG v1.0.1
  )
  FetchContent_MakeAvailable(utils)
  # Fetch types
  FetchContent_Declare(
    types
    GIT_REPOSITORY https://github.com/bolderflight/types.git
    GIT_TAG v0.5.0
  )
  FetchContent_MakeAvailable(types)
  # Add the library target
  add_library(invensense_imu
    src/invensense_imu.cpp
    src/invensense_imu.h
    src/mpu9250.cpp
    src/mpu9250.h
    src/mpu6500.cpp
    src/mpu6500.h
    src/bfs-mpu6500.h
    src/bfs-mpu6500.cpp
    src/bfs-mpu9250.h
    src/bfs-mpu9250.cpp
  )
  # Link libraries
  target_link_libraries(invensense_imu
    PUBLIC
      core
      eigen
      statistics
      types
      utils
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

    ### BFS-MPU6500
    add_executable(bfs_mpu6500_ex examples/cmake/bfs/mpu6500.cc)
    # Add the includes
    target_include_directories(bfs_mpu6500_ex PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(bfs_mpu6500_ex
      PRIVATE
        invensense_imu
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(bfs_mpu6500_ex ${MCU} ${mcu_support_SOURCE_DIR})

    ### BFS-MPU9250
    add_executable(bfs_mpu9250_ex examples/cmake/bfs/mpu9250.cc)
    # Add the includes
    target_include_directories(bfs_mpu9250_ex PUBLIC 
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    # Link libraries to the example target
    target_link_libraries(bfs_mpu9250_ex
      PRIVATE
        invensense_imu
    )
    # Add hex and upload targets
    include(${mcu_support_SOURCE_DIR}/cmake/flash_mcu.cmake)
    FlashMcu(bfs_mpu9250_ex ${MCU} ${mcu_support_SOURCE_DIR})
  endif()
endif()
