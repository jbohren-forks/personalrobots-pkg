cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)


rospack(outlet_spotting)

gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(outlet_spotting src/outlet_spotting.cpp src/outlet_tuple.cpp)
