cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(point_cloud_downsampler)
add_definitions(-Wall -O3 -DNDEBUG -march=pentium4 -mfpmath=sse -mmmx -msse -msse2 -msse3)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(cloud_downsampler_node src/cloud_downsampler.cpp)
target_link_libraries(cloud_downsampler_node lapack)
