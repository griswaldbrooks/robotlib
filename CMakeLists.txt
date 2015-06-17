cmake_minimum_required(VERSION 2.8)

# Project is called Robotlib
project (ROBOTLIB)
set(CMAKE_BUILD_TYPE Release)

# Set version numbers
set(ROBOTLIB_VERSION_MAJOR 0)
set(ROBOTLIB_VERSION_MINOR 1)

# Get Boost
find_package( Boost 1.40 COMPONENTS thread asio array )
include_directories( ${Boost_INCLUDE_DIR} )

# Include directory for Robotlib
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Libraries sources to build
set(_libsrcs
	src/rlib_state.cpp 
	src/rlib_actuator.cpp
	src/rlib_sensor.cpp
	src/rlib_robot.cpp
	src/rlib_nao.cpp
	src/hokuyo_client.cpp
	)

# Build these libraries
add_library(ROBOTLIB SHARED ${_libsrcs})