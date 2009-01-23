cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
rospack(executive_trex_pr2)

rospack_add_boost_directories()

execute_process(
    COMMAND rospack find trex
    ERROR_VARIABLE __rospack_err_ignore
    OUTPUT_VARIABLE trex_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE)

if(__rospack_err_ignore)
  message(FATAL_ERROR, "Failed to invoke rospack to find trex")
endif(__rospack_err_ignore)



execute_process(
    COMMAND ${PROJECT_SOURCE_DIR}/run-jam
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR})



set(TREX_FILES src/Executive.cc 
	       src/ROSAdapter.cc 
	       src/Logger.cc 
               src/LogClock.cc 
	       src/CalcAngleDiffConstraint.cc 
	       src/CalcDistanceConstraint.cc 
	       src/Components.cc
	       src/BaseStateAdapter.cc
	       src/BaseControllerAdapter.cc
	       src/ArmStateAdapter.cc
	       src/ArmControllerAdapter.cc
	       src/EndEffectorControllerAdapter.cc
	       src/RechargeControllerAdapter.cc
	       src/BatteryStateAdapter.cc)

# trexdebug builds with a large number of run-time error checking running which is expensive
# but gives good feedback in discovering problems.
rospack_add_executable(bin/trexdebug ${TREX_FILES})
target_link_libraries(bin/trexdebug Utils_g TREX_g)
rospack_link_boost(bin/trexdebug thread)
rospack_add_gtest_build_flags(bin/trexdebug)
# rospack_add_gtest_build_flags excludes the target from all, on the
# assumption that it's only used as a unit test.  In this case that's not
# true
set_target_properties(bin/trexdebug PROPERTIES EXCLUDE_FROM_ALL false)


# trexfast is about an order of magnitude faster than trexdebug
rospack_add_executable(bin/trexfast ${TREX_FILES})
target_link_libraries(bin/trexfast Utils_o TREX_o)
rospack_link_boost(bin/trexfast thread)
rospack_add_gtest_build_flags(bin/trexfast)
# rospack_add_gtest_build_flags excludes the target from all, on the
# assumption that it's only used as a unit test.  In this case that's not
# true
set_target_properties(bin/trexfast PROPERTIES EXCLUDE_FROM_ALL false)
rospack_add_compile_flags(bin/trexfast "-O3 -DEUROPA_FAST")

# trexwatchdog is a safety process that will monitor pings from the executive
# and dispense shutdown commands to highlevel controllers (deactivation) if pings are
# not received in time
rospack_add_executable(bin/trexwatchdog src/WatchDog.cc)

# Integrated test cases for the nav stack running with the pr2 executive and model.
rospack_add_rostest_future(wpc.0/test.xml)
rospack_add_rostest_future(wpc.1/test.xml)
rospack_add_rostest_future(wpc.2/test.xml)
rospack_add_rostest_future(miles.0/test.xml)

rospack_add_executable(test/groundtruthtransform test/groundtruthtransform.cpp)
