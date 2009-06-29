cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
rospack(trex_pr2)

genmsg()
gensrv()

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
rospack_add_nddl("test/doors/single_action_test/pr2.doorman.nddl")
rospack_add_nddl("test/doors/without_map/pr2.doorman.nddl")
rospack_add_nddl("test/doors/with_map/pr2.doorman.nddl")
rospack_add_nddl("test/doors/open_door.0/pr2.doorman.nddl")
rospack_add_nddl("test/doors/open_door.0/pr2.rcs.sim.nddl")
rospack_add_nddl("test/doors/open_door.1/pr2.doorman.nddl")
rospack_add_nddl("test/doors/open_door.1/pr2.rcs.sim.nddl")

# Plug Tests
rospack_add_nddl("test/plugs/pr2.rcs.sim.nddl")
rospack_add_nddl("test/plugs/single_action_test/pr2.recharger.nddl")
rospack_add_nddl("test/plugs/recharge.0/pr2.recharger.nddl")
rospack_add_nddl("test/plugs/recharge.1/pr2.recharger.nddl")

# Mechanism Control Tests
rospack_add_nddl("test/mechanism_control/pr2.mechanism_control.nddl")
rospack_add_nddl("test/mechanism_control/pr2.rcs.sim.nddl")

# Master Tests
rospack_add_nddl("test/master/pr2.rcs.sim.nddl")
rospack_add_nddl("test/master/pr2.driver.nddl")
rospack_add_nddl("test/master/pr2.doorman.nddl")
rospack_add_nddl("test/master/pr2.recharger.nddl")
rospack_add_nddl("test/master/domain_tests/master.0/pr2.master.nddl")
rospack_add_nddl("test/master/domain_tests/master.1/pr2.master.nddl")
rospack_add_nddl("test/master/domain_tests/master.2/pr2.master.nddl")
rospack_add_nddl("test/master/domain_tests/master.3/pr2.master.nddl")
rospack_add_nddl("test/master/domain_tests/master.4/pr2.master.nddl")
rospack_add_nddl("test/master/m2/pr2.master.nddl")
rospack_add_nddl("test/master/m2/no_plugs/pr2.rcs.sim.plugs.nddl")
rospack_add_nddl("test/master/m2/no_plugs/pr2.recharger.nddl")
rospack_add_nddl("test/master/m2/no_plugs/no_arms/pr2.mechanism_control.nddl")
rospack_add_nddl("test/master/m2/no_arms/pr2.mechanism_control.nddl")
rospack_add_nddl("test/master/m2/high_level_sim/pr2.rcs.sim.nddl")
rospack_add_nddl("test/master/m2_remote_interface/pr2.master.nddl")

set(TREX_FILES src/pr2_adapter_utilities.cpp
	       src/pr2_components.cpp
	       src/topological_map.cpp
	       src/door_domain_constraints.cpp
	       src/adapters.cpp
	       src/master_reactor.cpp)

# trexdebug builds with a large number of run-time error checking running which is expensive
# but gives good feedback in discovering problems.
rospack_add_library(executive_trex_pr2_g ${TREX_FILES})
target_link_libraries(executive_trex_pr2_g executive_trex_ros_g Utils_g TREX_g)
rospack_add_executable(bin/trexdebug src/main.cpp)
target_link_libraries(bin/trexdebug executive_trex_pr2_g )
rospack_link_boost(bin/trexdebug thread)
rospack_add_gtest_build_flags(bin/trexdebug)

# rospack_add_gtest_build_flags excludes the target from all, on the
# assumption that it's only used as a unit test.  In this case that's not
# true
#set_target_properties(bin/trexdebug PROPERTIES EXCLUDE_FROM_ALL false)

# trexfast is about an order of magnitude faster than trexdebug
rospack_add_library(executive_trex_pr2_o ${TREX_FILES})
target_link_libraries(executive_trex_pr2_o executive_trex_ros_o Utils_o TREX_o)
rospack_add_executable(bin/trexfast src/main.cpp)
target_link_libraries(bin/trexfast executive_trex_pr2_o)
rospack_link_boost(bin/trexfast thread)
rospack_add_gtest_build_flags(bin/trexfast)
# rospack_add_gtest_build_flags excludes the target from all, on the
# assumption that it's only used as a unit test.  In this case that's not
# true
set_target_properties(bin/trexfast PROPERTIES EXCLUDE_FROM_ALL false)
rospack_add_compile_flags(bin/trexfast "-O3 -DEUROPA_FAST")
rospack_add_compile_flags(executive_trex_pr2_o "-O3 -DEUROPA_FAST")





# Test target for unit tests on topological map integration components
rospack_add_gtest(test/topological_map_tests test/topological_map_tests.cpp)
target_link_libraries(test/topological_map_tests executive_trex_pr2_g)

# stub_ros__container implements stub state publishers and actions to allow easy testing of the ROS adapters for the model
rospack_add_executable(bin/stub_ros_container src/stub_ros_container.cpp)

# Implementation of actions that we actually use
rospack_add_executable(bin/actions src/actions.cpp)

# Try the API for running the service calls to the executive
rospack_add_executable(test/ros_reactor_test test/ros_reactor_test.cpp)

