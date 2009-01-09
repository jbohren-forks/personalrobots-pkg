cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(dynamic_planning)
include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(dynamic_planning 
                         src/discrete_space_information/nav3ddyn/environment_nav3Ddyn.cpp
			 )
# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(test_nav3ddyn src/test/nav3ddyn_test.cpp)
target_link_libraries(test_nav3ddyn dynamic_planning sbpl)

# Test target for module tests to be included in gtest regression test harness
rospack_add_gtest(utest test/utest.cpp)
target_link_libraries(utest dynamic_planning sbpl)

# Send output of tests to the test directory
set_target_properties(utest PROPERTIES
                      RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/test)

