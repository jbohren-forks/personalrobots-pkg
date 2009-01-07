cmake_minimum_required(VERSION 2.6)

include(rosbuild)
set(ROS_BUILD_TYPE Release)
rospack(normal_estimation)
add_definitions(-Wall)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(normal_estimation_node src/normal_estimation.cpp)
target_link_libraries(normal_estimation_node lapack)
