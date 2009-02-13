cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(topological_map)
gensrv()

rospack_add_boost_directories()

#uncomment for optimizing compilation
#set(ROS_COMPILE_FLAGS "-O3" ${ROS_COMPILE_FLAGS})

#uncomment for profiling/debugging
set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})




rospack_add_library(topological_map src/region.cpp)
