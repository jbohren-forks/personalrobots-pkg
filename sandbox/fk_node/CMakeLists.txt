cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(fk_node)

rospack_add_boost_directories()

genmsg()
gensrv()

rospack_add_executable(fk_node src/fk_node.cpp)

