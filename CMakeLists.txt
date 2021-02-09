cmake_minimum_required(VERSION 3.13)
if (DEFINED MCU)
	# Setting up the toolchain
	set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
endif()
# Project information
project(Mpu9250
	VERSION 2.0.1
	DESCRIPTION "MPU-9250 sensor driver"
	LANGUAGES CXX
)
if (DEFINED MCU)
	# Grab the processor and set up definitions and compile options
	include(${CMAKE_SOURCE_DIR}/cmake/config_mcu.cmake)
	configMcu(${MCU})
	include(FetchContent)
	# Fetch core
	FetchContent_Declare(
		core
		GIT_REPOSITORY 	https://github.com/bolderflight/core.git
		GIT_TAG v2.0.4
	)
	FetchContent_MakeAvailable(core)
	# Fetch eigen
	FetchContent_Declare(
		eigen
		GIT_REPOSITORY 	https://github.com/bolderflight/eigen.git
		GIT_TAG v1.0.0
	)
	FetchContent_MakeAvailable(eigen)
	# Fetch global defs
	FetchContent_Declare(
		units
		GIT_REPOSITORY 	https://github.com/bolderflight/units.git
		GIT_TAG v2.0.0
	)
	FetchContent_MakeAvailable(units)
	# Add the library target
	add_library(mpu9250
		src/mpu9250/mpu9250.cc
		include/mpu9250/mpu9250.h
	)
	# Link libraries
	target_link_libraries(mpu9250
		PUBLIC
			core
			eigen
			units
	)
	# Setup include directories 
	target_include_directories(mpu9250 PUBLIC 
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
	)
endif()

# Example and test if this project is built separately
if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
	if (DEFINED MCU)		
		# Add the spi example target
		add_executable(spi_example examples/spi.cc)
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
		include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
		FlashMcu(spi_example ${MCU})
		# Add the i2c example target
		add_executable(i2c_example examples/i2c.cc)
		# Add the includes
		target_include_directories(spi_example PUBLIC 
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<INSTALL_INTERFACE:include>
		)
		# Link libraries to the example target
		target_link_libraries(i2c_example
			PRIVATE 
				mpu9250
		)
		# Add hex and upload targets
		include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
		FlashMcu(i2c_example ${MCU})
	endif()
endif()
