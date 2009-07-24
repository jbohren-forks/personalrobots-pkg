cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(image_segmentation)

set(ROS_BUILD_TYPE Release)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rospack_add_executable(msfhseg_node src/msfhseg_node.cpp)

rospack_add_library(meanshift_generic src/meanshift_generic/meanshift_generic.cpp)
rospack_add_executable(meanshift_example examples/meanshift_example.cpp)
target_link_libraries(meanshift_example meanshift_generic)

