cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(sbpl_arm_executive)
gensrv()

# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rospack_add_library(pr2_arm_node src/pr2_arm_node.cpp)

rospack_add_executable(sbpl_arm_executive src/sbpl_arm_executive.cpp)
target_link_libraries(sbpl_arm_executive pr2_arm_node)
