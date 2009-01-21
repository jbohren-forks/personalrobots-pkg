cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(cloud_geometry)
add_definitions(-Wall)

rospack_add_library (cloud_geometry src/nearest.cpp src/point.cpp src/distances.cpp src/intersections.cpp src/areas.cpp src/transforms.cpp)
