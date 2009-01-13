cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(dynamic_planning)
include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(dynamic_planning 
                         src/discrete_space_information/nav3ddyn/environment_nav3Ddyn.cpp
			 )
rospack_add_executable(bin/test_nav3ddyn src/test/nav3ddyn_test.cpp)
target_link_libraries(bin/test_nav3ddyn dynamic_planning sbpl)

# Test target for module tests to be included in gtest regression test harness
rospack_add_gtest(test/utest test/utest.cpp)
target_link_libraries(test/utest dynamic_planning sbpl)
