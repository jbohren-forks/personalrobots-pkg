cmake_minimum_required(VERSION 2.6)
include(rosbuild)
add_definitions(-Wall)

rospack(overhead_grasp_behavior)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(overhead_grasp src/overhead_grasp.cc)


