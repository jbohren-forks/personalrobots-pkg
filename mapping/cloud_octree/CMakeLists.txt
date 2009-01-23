cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(cloud_octree)
add_definitions(-Wall)
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(cloud_octree src/octree.cpp src/octree_node.cpp)
#src/octree_ros.cpp)

#rospack_add_gtest(test_octree src/octree_ros.cpp)
#target_link_libraries(test_octree)
