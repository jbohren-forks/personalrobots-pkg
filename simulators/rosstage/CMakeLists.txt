cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(rosstage)

rospack_add_boost_directories()

rospack_add_executable(rosstage rosstage.cc)
rospack_link_boost(rosstage thread)

rospack_add_rostest(test/hztest.xml)
