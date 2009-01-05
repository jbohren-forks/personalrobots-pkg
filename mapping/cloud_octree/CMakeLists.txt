cmake_minimum_required(VERSION 2.6)

include(rosbuild)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(cloud_octree)
add_definitions(-Wall)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(cloud_octree src/octree.cpp src/octree_node.cpp)
#src/octree_ros.cpp)

#rospack_add_gtest(test_octree src/octree_ros.cpp)
#target_link_libraries(test_octree)
