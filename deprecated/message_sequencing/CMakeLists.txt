cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(message_sequencing)
ADD_DEFINITIONS(-D__STDC_LIMIT_MACROS -DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)

rospack_add_boost_directories()

rospack_add_library(messageseq src/allocation.cpp)
rospack_link_boost(messageseq thread)

#add_subdirectory(example)
