cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(topic_synchronizer)
ADD_DEFINITIONS(-D__STDC_LIMIT_MACROS -DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)
rospack_add_library(messageseq src/allocation.cpp)

add_subdirectory(example)
