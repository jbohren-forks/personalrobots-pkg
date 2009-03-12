cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(test_executive_trex_pr2)


################################################################################
# This whole section could be included. It is used for building jam targets
################################################################################
execute_process(
    COMMAND rospack find trex
    ERROR_VARIABLE __rospack_err_ignore
    OUTPUT_VARIABLE trex_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE)

if(__rospack_err_ignore)
  message(FATAL_ERROR, "Failed to invoke rospack to find trex")
endif(__rospack_err_ignore)

execute_process(
    COMMAND rospack find executive_trex_pr2
    ERROR_VARIABLE __rospack_err_ignore
    OUTPUT_VARIABLE exec_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE)

if(__rospack_err_ignore)
  message(FATAL_ERROR, "Failed to invoke rospack to find trex")
endif(__rospack_err_ignore)

execute_process(
    COMMAND jam ${ROS_PARALLEL_JOBS} -sTREX_PKG_ROOT=${trex_dir} -sEXEC_HOME=${exec_dir} exec_inputs
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    RESULT_VARIABLE JAM_RET_VAR)
 
if (JAM_RET_VAR)
   message(FATAL_ERROR "Jam failed with error code: " ${JAM_RET_VAR} ". If jam is not installed, you will need it. Try 'apt-get install jam'.")
endif (JAM_RET_VAR)
################################################################################


# Integrated test cases for the nav stack running with the pr2 executive and model.
#rospack_add_rostest(wpc.0/test.xml) 
#rospack_add_rostest(wpc.1/test.xml) 
#rospack_add_rostest(wpc.2/test.xml) 
#rospack_add_rostest(wpc.3/test.xml) 


#rospack_add_rostest_graphical(miles.0/test.xml)
#rospack_add_rostest_future(miles.0/test.xml) 
#rospack_add_rostest_graphical(miles.2/test.xml)
#rospack_add_rostest_future(miles.2/test.xml) 
#rospack_add_rostest_graphical(arm.0/launch.xml)
#rospack_add_rostest_future(arm.0/launch.xml)
