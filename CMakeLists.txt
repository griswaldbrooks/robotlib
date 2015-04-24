cmake_minimum_required(VERSION 2.8)

# Project is called Robotlib
project (ROBOTLIB)
set(CMAKE_BUILD_TYPE Release)

# Set version numbers
set (ROBOTLIB_VERSION_MAJOR 0)
set (ROBOTLIB_VERSION_MINOR 1)

# Include directory for Robotlib
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Libraries sources to build
set(_libsrcs
	src/rlib_state.cpp 
	src/rlib_actuator.cpp
	src/rlib_nao.cpp
	)

# Build these libraries
add_library(ROBOTLIB SHARED ${_libsrcs})