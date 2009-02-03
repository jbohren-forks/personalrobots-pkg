cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_2dnav_gazebo)

# This test file isn't present; perhaps a cut-n-paste typo?
#rospack_add_rostest(test_pendulum.xml)
rospack_add_rostest(test_2dnav_wg.xml)
rospack_add_rostest(test_2dnav_empty_rotation.xml)
rospack_add_rostest(test_2dnav_empty_odom.xml)
rospack_add_rostest(test_2dnav_empty_diagonal.xml)
rospack_add_rostest(test_2dnav_empty_axis.xml)

