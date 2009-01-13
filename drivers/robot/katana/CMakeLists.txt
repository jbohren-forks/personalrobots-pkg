cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(katana)
gensrv()
add_subdirectory(libkatana)
add_subdirectory(standalone)
add_subdirectory(nodes)

