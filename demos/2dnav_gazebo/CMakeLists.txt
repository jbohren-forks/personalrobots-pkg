cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(2dnav_gazebo)

rospack_add_rostest(test/test_2dnav_empty.xml)
rospack_add_rostest(test/test_2dnav_wg.xml)

