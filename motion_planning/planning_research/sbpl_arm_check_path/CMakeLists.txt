cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(sbpl_arm_check_path)
gensrv()

# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(sbpl_arm_check_path src/sbpl_arm_check_path.cpp)
