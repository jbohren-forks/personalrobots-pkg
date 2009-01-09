cmake_minimum_required(VERSION 2.6)

include(rosbuild)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(semantic_point_annotator)
add_definitions(-Wall)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(semantic_point_annotator_node src/semantic_point_annotator.cpp)
target_link_libraries(semantic_point_annotator_node lapack)
