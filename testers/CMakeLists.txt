cmake_minimum_required(VERSION 2.8)
project(rlib_testers)

# Set tester include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Bring Robotlib into the fold
set(ROBOTLIB_DIR /home/crrl-user1/code_projects/robotlib)
include_directories("${ROBOTLIB_DIR}/include")
link_directories("${ROBOTLIB_DIR}/lib")

# Get Boost
set(ROBOTLIB_BOOSTLIBS boost_system boost_thread pthread)
find_package( Boost 1.40 )
include_directories( ${Boost_INCLUDE_DIR} )

# Create tester executables
add_executable(test_actuator src/rlib_actuator_tester.cpp)
add_executable(test_nao src/rlib_nao_tester.cpp)
add_executable(test_navigation src/rlib_navigation_tester.cpp)

# Add libraries
target_link_libraries(test_actuator ROBOTLIB ${ROBOTLIB_BOOSTLIBS})
target_link_libraries(test_nao ROBOTLIB ${ROBOTLIB_BOOSTLIBS})
target_link_libraries(test_navigation ROBOTLIB ${ROBOTLIB_BOOSTLIBS})
