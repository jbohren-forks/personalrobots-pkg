cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_pr2_mechanism_controllers_gazebo)

rospack_add_rostest(test_base.xml)
rospack_add_rostest_future(test_arm.xml)

rospack_add_rostest(hztest_pr2_mechanism.xml)
rospack_add_rostest(hztest_rostime.xml)
rospack_add_rostest(hztest_pr2_odom.xml)

