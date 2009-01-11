cmake_minimum_required(VERSION 2.6)

include(rosbuild)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(collision_map)
add_definitions(-Wall -msse3)
genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(collision_map_node src/collision_map.cpp)
target_link_libraries(collision_map_node lapack)
