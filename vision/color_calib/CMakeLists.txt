cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(color_calib)
genmsg()

rospack_add_boost_directories()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_library(colorcalib libcolorcalib.cpp calibration.cpp)
rospack_add_executable(calib_test calib_test.cpp)
target_link_libraries(calib_test colorcalib)

rospack_add_executable(calib_node calib_node.cpp)
target_link_libraries(calib_node colorcalib)
rospack_link_boost(calib_node thread)

rospack_add_executable(calib_node_dcam calib_node_dcam.cpp)
target_link_libraries(calib_node_dcam colorcalib)
rospack_link_boost(calib_node_dcam thread)
