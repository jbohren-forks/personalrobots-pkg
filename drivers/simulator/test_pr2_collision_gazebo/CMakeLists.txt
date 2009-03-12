cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_pr2_collision_gazebo)

rospack_add_rostest_graphical(test_slide.xml)

