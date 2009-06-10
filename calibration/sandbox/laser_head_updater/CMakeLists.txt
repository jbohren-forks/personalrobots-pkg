cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(laser_head_updater)

rospack_add_executable(publish_laser_head_tf src/publish_laser_head_tf.cpp)
rospack_add_executable(publish_laser_head_tf_pre src/publish_laser_head_tf_pre.cpp)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

