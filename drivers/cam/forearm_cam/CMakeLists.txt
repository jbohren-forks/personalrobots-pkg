cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE RelWithDebInfo)
rospack(forearm_cam)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

gensrv()
rospack_add_boost_directories()

rospack_add_rostest(test/test_with_sim.xml)

add_subdirectory(src)
