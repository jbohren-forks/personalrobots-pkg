cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(2dnav_gazebo)

rospack_add_rostest(test/test_2dnav_empty_axis.xml)
rospack_add_rostest(test/test_2dnav_empty_diagonal.xml)
rospack_add_rostest(test/test_2dnav_empty_rotation.xml)
rospack_add_rostest(test/test_2dnav_empty_odom.xml)
rospack_add_rostest(test/test_2dnav_wg.xml)

