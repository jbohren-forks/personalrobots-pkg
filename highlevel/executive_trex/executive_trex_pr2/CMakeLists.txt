cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
rospack(executive_trex_pr2)


genmsg()

rospack_add_boost_directories()

# To build exec input files
setup_nddl()

# Shared files
rospack_add_nddl("test/pr2.safety.nddl")
rospack_add_nddl("test/pr2.mechanism_control.nddl")

# Nav Tests
rospack_add_nddl("test/nav/pr2.driver.nddl")
rospack_add_nddl("test/nav/sim/move.0/pr2.rcs.sim.nddl")

# Door Tests
rospack_add_nddl("test/doors/pr2.rcs.sim.nddl")
rospack_add_nddl("test/doors/single_action_test/pr2.doorman.nddl")
rospack_add_nddl("test/doors/without_map/pr2.doorman.nddl")
rospack_add_nddl("test/doors/with_map/pr2.doorman.nddl")
rospack_add_nddl("test/doors/open_door.0/pr2.doorman.nddl")
rospack_add_nddl("test/doors/open_door.0/pr2.rcs.sim.nddl")

# Plug Tests
rospack_add_nddl("test/plugs/single_action_test/pr2.recharger.nddl")
rospack_add_nddl("test/plugs/recharge.0/pr2.recharger.nddl")
rospack_add_nddl("test/plugs/recharge.0/pr2.rcs.sim.nddl")

# Mechanism Control Tests
rospack_add_nddl("test/mechanism_control/pr2.mechanism_control.nddl")
rospack_add_nddl("test/mechanism_control/pr2.rcs.sim.nddl")

# Master Tests
rospack_add_nddl("test/master/pr2.master.nddl")
rospack_add_nddl("test/master/pr2.sim.nddl")

#rospack_add_nddl("test/model_tests/pr2.master.nddl")
#rospack_add_nddl("test/model_tests/pr2.doorman.nddl")
#rospack_add_nddl("test/model_tests/pr2.driver.nddl")
#rospack_add_nddl("test/model_tests/pr2.recharger.nddl")
#rospack_add_nddl("test/model_tests/pr2.safety.nddl")
#rospack_add_nddl("test/model_tests/pr2.rcs.sim.nddl")

# Variant for basic door controller test
#rospack_add_nddl("test/model_tests/doorman/test_basic_controller/pr2.doorman.nddl")
#rospack_add_nddl("test/model_tests/doorman/test_basic_controller/pr2.rcs.sim.nddl")

# Variant for basic message flow with action adapters
#rospack_add_nddl("test/model_tests/doorman/test_basic_message_flow/pr2.doorman.nddl")
#rospack_add_nddl("test/model_tests/doorman/test_basic_message_flow/pr2.rcs.sim.nddl")

#rospack_add_nddl("test/pr2.test.nddl")
#rospack_add_nddl("wpc_gazebo.0/pr2.exec.nddl")
#rospack_add_nddl("wpc_gazebo.1/pr2.exec.nddl")
#rospack_add_nddl("nav.0/pr2.exec.nddl")
#rospack_add_nddl("arm.0/pr2.exec.nddl")
#rospack_add_nddl("arm.1/pr2.exec.nddl")
#rospack_add_nddl("arm.2/pr2.exec.nddl")
#rospack_add_nddl("miles.1/pr2.exec.nddl")
#rospack_add_nddl("miles.2/pr2.exec.nddl")
#rospack_add_nddl("miles.3/pr2.exec.nddl")
#rospack_add_nddl("miles_gazebo.3/pr2.exec.nddl")
#rospack_add_nddl("miles.4/pr2.exec.nddl")
#rospack_add_nddl("sim.0/pr2.exec.nddl")
#rospack_add_nddl("sim.0/pr2.sim.nddl")
#rospack_add_nddl("sim.1/pr2.exec.nddl")
#rospack_add_nddl("sim.1/pr2.sim.nddl")
#rospack_add_nddl("master.0/pr2.master.nddl")
#rospack_add_nddl("master.0/pr2.doorman.nddl")
#rospack_add_nddl("master.0/pr2.driver.nddl")
#rospack_add_nddl("master.0/pr2.recharger.nddl")
#rospack_add_nddl("master.0/pr2.safety.nddl")
#rospack_add_nddl("master.0/pr2.rcs.sim.nddl")
#rospack_add_nddl("doorman.0/pr2.master.nddl")
#rospack_add_nddl("doorman.0/pr2.doorman.nddl")
#rospack_add_nddl("doorman.0/pr2.driver.nddl")
#rospack_add_nddl("doorman.0/pr2.recharger.nddl")
#rospack_add_nddl("doorman.0/pr2.safety.nddl")
#rospack_add_nddl("doorman.0/pr2.rcs.sim.nddl")
#rospack_add_nddl("doorman.1/pr2.master.nddl")
#rospack_add_nddl("doorman.1/pr2.doorman.nddl")
#rospack_add_nddl("doorman.1/pr2.driver.nddl")
#rospack_add_nddl("doorman.1/pr2.recharger.nddl")
#rospack_add_nddl("doorman.1/pr2.safety.nddl")
#rospack_add_nddl("doorman.1/pr2.rcs.sim.nddl")
#rospack_add_nddl("plugs.0/pr2.recharger.nddl")
#rospack_add_nddl("plugs.0/pr2.safety.nddl")
#rospack_add_nddl("plugs.0/pr2.rcs.sim.nddl")

set(TREX_FILES src/executive.cc
	       src/monitor.cc
	       src/ros_adapter.cc 
	       src/logger.cc 
               src/logclock.cc 
	       src/calc_angle_diff_constraint.cc 
	       src/calc_distance_constraint.cc 
	       src/components.cc
	       src/base_state_adapter.cc
	       src/arm_state_adapter.cc
	       src/arm_controller_adapter.cc
	       src/end_effector_controller_adapter.cc
	       src/battery_state_adapter.cc
	       src/topological_map.cc
	       src/door_domain_constraints.cc
	       src/adapter_utilities.cc
	       src/adapters.cc)

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
target_link_libraries(executive_trex_pr2_o Utils_o TREX_o topological_graph)
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
rospack_add_executable(bin/trexwatchdog src/watch_dog.cc)

# Test target for unit tests on topological map integration components
rospack_add_gtest(test/topological_map_tests test/topological_map_tests.cc)
target_link_libraries(test/topological_map_tests executive_trex_pr2_g)

# stub_ros__container impements stub state publishers and actions to allow easy testing of the ROS adapters for the model
rospack_add_executable(bin/stub_ros_container src/stub_ros_container.cc)

#rospack_add_executable(test/groundtruthtransform test/groundtruthtransform.cpp)
