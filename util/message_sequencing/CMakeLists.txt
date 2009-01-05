cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(topic_synchronizer)
ADD_DEFINITIONS(-D__STDC_LIMIT_MACROS -DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)
rospack_add_library(messageseq src/allocation.cpp)

add_subdirectory(example)
