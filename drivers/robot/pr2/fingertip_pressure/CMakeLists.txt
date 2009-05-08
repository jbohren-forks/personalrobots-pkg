cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(fingertip_pressure)
genmsg()

add_definitions(-Wall)

rospack_add_rostest(test/pressure_info_tests.xml)
rospack_add_rostest(test/test_marker.xml)
rospack_add_rostest(test/test_marker_sphere.xml)
