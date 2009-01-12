cmake_minimum_required(VERSION 2.6)

include(rosbuild)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(stereo_convex_patch_histogram)
add_definitions(-Wall)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(convex_patch_histogram_node src/convex_patch_histogram.cpp)
target_link_libraries(convex_patch_histogram_node lapack)
