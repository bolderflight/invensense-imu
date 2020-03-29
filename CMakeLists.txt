cmake_minimum_required(VERSION 3.13)
# Setting up the toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
# Project information
project(MPU9250
  VERSION 1.0
  DESCRIPTION "MPU-9250 sensor driver"
  LANGUAGES C CXX
)
# Grab the processor and set up definitions and compile options
include(${CMAKE_SOURCE_DIR}/cmake/config_mcu.cmake)
configMcu(${MCU})
# Fetch types
include(FetchContent)
FetchContent_Declare(
	types
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/types.git
	GIT_TAG					v1.0.0
)
FetchContent_MakeAvailable(types)
# Fetch core
FetchContent_Declare(
	core
	GIT_REPOSITORY 	git@gitlab.com:bolderflight/software/core.git
	GIT_TAG					v0.8.0
)
FetchContent_MakeAvailable(core)
# Add the library target
add_library(mpu9250
  src/mpu9250/mpu9250.cc
  include/mpu9250/mpu9250.h
)
# Link libraries
target_link_libraries(mpu9250
	PUBLIC
    core
    types
)
# Setup include directories 
target_include_directories(mpu9250 PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 	$<INSTALL_INTERFACE:include>
)
# Example if this project is built separately
if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
  # Add the example target
	add_executable(i2c_example examples/i2c.cc)
	# Add the includes
	target_include_directories(i2c_example PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	)
	# Link libraries to the test target
	target_link_libraries(i2c_example
		PRIVATE 
      mpu9250
  )
  # Add hex and upload targets
  include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
  FlashMcu(i2c_example ${MCU})
endif()
