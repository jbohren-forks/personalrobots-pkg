cmake_minimum_required(VERSION 2.6)

include(rosbuild)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(cloud_kdtree)
add_definitions(-Wall)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(cloud_kdtree src/convert.cpp src/search.cpp)

rospack_add_gtest(test_kdtree test/test_kdtree.cpp)
target_link_libraries(test_kdtree cloud_kdtree)
