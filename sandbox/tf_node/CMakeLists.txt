cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ROS_BUILD_TYPE Release)
rospack(tf_node)

rospack_add_boost_directories()

genmsg()
gensrv()

rospack_add_executable(tf_node src/tf_node.cpp)

