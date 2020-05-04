cmake_minimum_required(VERSION 3.13)
if (DEFINED MCU)
	# Setting up the toolchain
	set(CMAKE_TOOLCHAIN_FILE "${CMAKE_SOURCE_DIR}/cmake/cortex.cmake")
endif()
# Project information
project(MPU9250
	VERSION 1.0
	DESCRIPTION "MPU-9250 sensor driver"
	LANGUAGES C CXX
)
if (DEFINED MCU)
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
endif()
# Example and test if this project is built separately
if (PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME)
	if (DEFINED MCU)
		# Fetch the HIL defs
		FetchContent_Declare(
			mcu_hil_defs
			GIT_REPOSITORY 	git@gitlab.com:bolderflight/utils/mcu_hil_defs.git
		)
		FetchContent_MakeAvailable(mcu_hil_defs)		
		# Add the example target
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
				mcu_hil_defs
		)
		# Add hex and upload targets
		include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
		FlashMcu(spi_example ${MCU})
		# Add the test target
		# Fetch remote_test 
		FetchContent_Declare(
			remote_test 
			GIT_REPOSITORY 	git@gitlab.com:bolderflight/utils/remote_test.git
		)
		FetchContent_MakeAvailable(remote_test)
		add_executable(slave tests/slave.cc)
		# Add the includes
		target_include_directories(slave PUBLIC 
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<INSTALL_INTERFACE:include>
		)
		# Link libraries to the test target
		target_link_libraries(slave
			PRIVATE
				remote_test 
				mpu9250
				mcu_hil_defs
		)
		# Add hex and upload targets
		include(${CMAKE_SOURCE_DIR}/cmake/flash_mcu.cmake)
		FlashMcu(slave ${MCU})
	else()
		# Fetch google test
		include(FetchContent)
		FetchContent_Declare(
			googletest
			GIT_REPOSITORY 	https://github.com/google/googletest.git
			GIT_TAG					release-1.10.0
		)
		FetchContent_MakeAvailable(googletest)
		enable_testing()
		include(GoogleTest)
		# Fetch remote_test 
		FetchContent_Declare(
			remote_test 
			GIT_REPOSITORY 	git@gitlab.com:bolderflight/utils/remote_test.git
		)
		FetchContent_MakeAvailable(remote_test)
		# Add the test target
		add_executable(master tests/master.cc)
		# Add the includes
		target_include_directories(master PUBLIC 
			$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
			$<INSTALL_INTERFACE:include>
		)
		# Link libraries to the test target
		target_link_libraries(master
			PRIVATE 
				remote_test
				gtest_main 
				gtest
				gmock
		)
		# Discover unit tests
		gtest_discover_tests(master)
	endif()
endif()
