cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(vision_utils)

find_package(PythonLibs)
include_directories(${PYTHON_INCLUDE_PATH})
