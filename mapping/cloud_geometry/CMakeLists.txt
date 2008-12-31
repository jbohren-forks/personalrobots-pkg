cmake_minimum_required(VERSION 2.6)

include(rosbuild)
set(ROS_BUILD_TYPE Release)
rospack(cloud_geometry)
add_definitions(-Wall)

rospack_add_library(cloud_geometry src/lapack.cpp src/nearest.cpp src/point.cpp src/distances.cpp src/intersections.cpp src/areas.cpp)
