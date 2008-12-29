cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(ivcon)

#uncomment for profiling
set(ROS_LINK_FLAGS "-lm" ${ROS_LINK_FLAGS})

rospack_add_executable(bin/ivcon src/ivcon.c)

