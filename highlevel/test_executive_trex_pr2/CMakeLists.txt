cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_executive_trex_pr2)


execute_process(
    COMMAND ${PROJECT_SOURCE_DIR}/run-jam
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    RESULT_VARIABLE JAM_RET_VAR)

if (JAM_RET_VAR)
   message(FATAL_ERROR "Jam failed with error code: " ${JAM_RET_VAR})
endif (JAM_RET_VAR)

#Tests commented awaiting Conor's fix for goal vs. condition
#functionality in TREX.

# Integrated test cases for the nav stack running with the pr2 executive and model.
#rospack_add_rostest(wpc.0/test.xml) #Commented awaiting Conor's fix.
#rospack_add_rostest(wpc.1/test.xml) #Commented awaiting Conor's fix.
#rospack_add_rostest(wpc.2/test.xml) #Commented awaiting Conor's fix.
#rospack_add_rostest(wpc.3/test.xml) #Commented awaiting Conor's fix.


#rospack_add_rostest_graphical(miles.0/test.xml)
#rospack_add_rostest_future(miles.0/test.xml) #Commented awaiting Conor's fix.
#rospack_add_rostest_graphical(miles.2/test.xml)
#rospack_add_rostest_future(miles.2/test.xml) #Commented awaiting Conor's fix.
#rospack_add_rostest_graphical(arm.0/launch.xml)
#rospack_add_rostest_future(arm.0/launch.xml) #Commented awaiting Conor's fix.
