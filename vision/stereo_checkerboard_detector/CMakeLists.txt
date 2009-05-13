cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(stereo_checkerboard_detector)

genmsg()

rospack_add_boost_directories()
rospack_add_library(stereo_checkerboard_detector src/mono_checkerboard_helper.cpp
                                              	 src/reprojection_helper.cpp
                                              	 src/stereo_checkerboard_helper.cpp)

rospack_add_executable(stereo_checkerboard_node src/stereo_checkerboard_node.cpp)
target_link_libraries(stereo_checkerboard_node stereo_checkerboard_detector)

rospack_add_executable(image_pc_debugger src/image_pc_debugger.cpp)
target_link_libraries(image_pc_debugger stereo_checkerboard_detector)

add_subdirectory(test EXCLUDE_FROM_ALL)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)