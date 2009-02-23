cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_pr2_sensors_gazebo)

rospack_add_rostest(test_scan.xml)
rospack_add_rostest(hztest_pr2_scan.xml)

# below need xvfb-run to test on build machine
rospack_add_rostest_graphical(test_camera.xml)
# This test is moved into the future, pending resolution of xvfb issues
# (#957)
#rospack_add_rostest_graphical(hztest_pr2_image.xml)
rospack_add_rostest_future(hztest_pr2_image.xml)

