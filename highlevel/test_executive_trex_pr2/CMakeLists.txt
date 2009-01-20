cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_executive_trex_pr2)


execute_process(
    COMMAND ${PROJECT_SOURCE_DIR}/run-jam
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

# Integrated test cases for the nav stack running with the pr2 executive and model.
rospack_add_rostest_graphical(wpc.0/test.xml)
rospack_add_rostest_graphical(wpc.1/test.xml)
#rospack_add_rostest_graphical(wpc.2/test.xml)
rospack_add_rostest_graphical(wpc.3/test.xml)
