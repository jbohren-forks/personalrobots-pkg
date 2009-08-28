cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)
rospack(pr2_mechanism_control)
genmsg()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_boost_directories()

rospack_add_library(pr2_mechanism_control src/mechanism_control.cpp src/recorder.cpp src/scheduler.cpp)
rospack_remove_compile_flags(pr2_mechanism_control -W)
rospack_link_boost(pr2_mechanism_control thread)
#rospack_add_executable(ms_publisher_test test/ms_publisher_test.cpp)
#rospack_add_rostest(test/test-mechanism-state-cpp.xml)
#rospack_add_rostest(test/test-mechanism-state-py.xml)

rospack_add_executable(action_runner src/action_mechanism_control.cpp)

