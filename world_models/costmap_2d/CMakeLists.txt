cmake_minimum_required(VERSION 2.6)
include(rosbuild)
set(ROS_BUILD_TYPE Release)
rospack(costmap_2d)
genmsg()

#uncomment for profiling
#set(ROS_COMPILE_FLAGS "-g -pg" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g -pg" ${ROS_LINK_FLAGS})
#set(ROS_COMPILE_FLAGS "-g" ${ROS_COMPILE_FLAGS})
#set(ROS_LINK_FLAGS "-g" ${ROS_LINK_FLAGS})

rospack_add_library(costmap_2d src/costmap_2d.cpp src/obstacle_map_accessor.cc)

# Test target for module tests to be included in gtest regression test harness
rospack_add_gtest(utest src/test/module-tests.cc)
target_link_libraries(utest costmap_2d)

# Target for benchmarking the costmap
rospack_add_executable(benchmark src/test/benchmark.cc src/costmap_2d.cpp src/obstacle_map_accessor.cc)
