cmake_minimum_required(VERSION 2.8)
project(rlib_testers)

# Set tester include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# Bring Robotlib into the fold
set(ROBOTLIB_DIR /home/crrl-user1/code_projects/robotlib)
include_directories("${ROBOTLIB_DIR}/include")
link_directories("${ROBOTLIB_DIR}/lib")

# Create tester executables
add_executable(test_actuator src/rlib_actuator_tester.cpp)
add_executable(test_nao src/rlib_nao_tester.cpp)

# Add libraries
target_link_libraries(test_actuator ROBOTLIB)
target_link_libraries(test_nao ROBOTLIB)
