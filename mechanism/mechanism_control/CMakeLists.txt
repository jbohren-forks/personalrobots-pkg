cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(mechanism_control)
genmsg()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()

rospack_add_library(mechanism_control src/mechanism_control.cpp)
rospack_remove_compile_flags(mechanism_control -W)
rospack_link_boost(mechanism_control thread)
#rospack_add_executable(ms_publisher_test test/ms_publisher_test.cpp)
#rospack_add_rostest(test/test-mechanism-state-cpp.xml)
#rospack_add_rostest(test/test-mechanism-state-py.xml)

rospack_add_executable(action_runner src/action_runner.cpp
				     src/action_mechanism_control.cpp)
rospack_add_executable(action_runner_test test/action_runner_test.cpp
					  src/action_mechanism_control.cpp)

