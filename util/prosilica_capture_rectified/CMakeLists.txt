cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(prosilica_capture_rectified)
rospack_add_boost_directories()

#gensrv()
#genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rospack_add_executable (prosilica_capture src/prosilica_capture.cpp)
