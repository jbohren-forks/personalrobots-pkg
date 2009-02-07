cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(object_database)

#rospack_add_boost_directories()

#uncomment for profiling
set(ROS_COMPILE_FLAGS "-g -pg -O3" ${ROS_COMPILE_FLAGS})
set(ROS_LINK_FLAGS "-g -pg -O3" ${ROS_LINK_FLAGS})

rospack_add_library(object_database src/object_database.cpp src/convex_polygon.cpp src/half_plane.cpp)

rospack_add_gtest(test/object_db_test test/object_db_test.cpp)
target_link_libraries(test/object_db_test object_database)
