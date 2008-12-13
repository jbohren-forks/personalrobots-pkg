cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(test_executive_trex_pr2)


execute_process(
    COMMAND ${PROJECT_SOURCE_DIR}/run-jam
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})

# Integrated test cases for the nav stack running with the pr2 executive and model.
rospack_add_rostest_future(wpc.0/test.xml)
rospack_add_rostest_future(wpc.1/test.xml)
rospack_add_rostest_future(wpc.2/test.xml)
rospack_add_rostest_future(miles.0/test.xml)
