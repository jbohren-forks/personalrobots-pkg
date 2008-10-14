cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(rosstage)
rospack_add_executable(rosstage rosstage.cc)

rospack_add_rostest_future(test/hztest.xml)
