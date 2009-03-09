cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
rospack(executive_trex_pr2)

genmsg()

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
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}
    RESULT_VARIABLE JAM_RET_VAR)
 
if (JAM_RET_VAR)
   message(FATAL_ERROR "Jam failed with error code: " ${JAM_RET_VAR})
endif (JAM_RET_VAR)



set(TREX_FILES src/Executive.cc
	       src/Monitor.cc
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
	       src/BatteryStateAdapter.cc
	       src/topological_map.cc
	       src/door_domain_constraints.cc)

# trexdebug builds with a large number of run-time error checking running which is expensive
# but gives good feedback in discovering problems.
rospack_add_library(executive_trex_pr2_g ${TREX_FILES})
target_link_libraries(executive_trex_pr2_g Utils_g TREX_g topological_graph)
rospack_add_executable(bin/trexdebug src/main.cc)
target_link_libraries(bin/trexdebug executive_trex_pr2_g )
rospack_link_boost(bin/trexdebug thread)
rospack_add_gtest_build_flags(bin/trexdebug)
# rospack_add_gtest_build_flags excludes the target from all, on the
# assumption that it's only used as a unit test.  In this case that's not
# true
set_target_properties(bin/trexdebug PROPERTIES EXCLUDE_FROM_ALL false)

# trexfast is about an order of magnitude faster than trexdebug
rospack_add_library(executive_trex_pr2_o ${TREX_FILES})
#target_link_libraries(executive_trex_pr2_o Utils_o TREX_o topological_graph)
target_link_libraries(executive_trex_pr2_o Utils_o TREX_o topological_graph)
rospack_add_executable(bin/trexfast src/main.cc)
target_link_libraries(bin/trexfast executive_trex_pr2_o)
rospack_link_boost(bin/trexfast thread)
rospack_add_gtest_build_flags(bin/trexfast)
# rospack_add_gtest_build_flags excludes the target from all, on the
# assumption that it's only used as a unit test.  In this case that's not
# true
set_target_properties(bin/trexfast PROPERTIES EXCLUDE_FROM_ALL false)
rospack_add_compile_flags(bin/trexfast "-O3 -DEUROPA_FAST")
rospack_add_compile_flags(executive_trex_pr2_o "-O3 -DEUROPA_FAST")

# trexwatchdog is a safety process that will monitor pings from the executive
# and dispense shutdown commands to highlevel controllers (deactivation) if pings are
# not received in time
rospack_add_executable(bin/trexwatchdog src/WatchDog.cc)

# Test target for unit tests on topological map integration components
rospack_add_gtest(test/topological_map_tests test/topological_map_tests.cc)
target_link_libraries(test/topological_map_tests executive_trex_pr2_g)

rospack_add_executable(test/groundtruthtransform test/groundtruthtransform.cpp)
