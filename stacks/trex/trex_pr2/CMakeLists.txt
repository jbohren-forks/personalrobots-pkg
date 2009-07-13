cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
rospack(trex_pr2)

genmsg()
gensrv()

rospack_add_boost_directories()

find_ros_package(trex_ros)
include(${trex_ros_PACKAGE_PATH}/NDDL.cmake)
create_nddl_config()


set(TREX_FILES src/pr2_adapter_utilities.cpp
	       src/pr2_components.cpp
	       src/topological_map.cpp
	       src/door_domain_constraints.cpp
	       src/pr2_adapters.cpp
	       src/master_reactor.cpp)

# build extensions into exported library
rospack_add_library(trex_pr2 ${TREX_FILES})

# trexdebug builds with a large number of run-time error checking running which is expensive
# but gives good feedback in discovering problems.
rospack_add_executable(bin/trexdebug src/main.cpp)
rospack_link_boost(bin/trexdebug thread)
rospack_add_gtest_build_flags(bin/trexdebug)
rospack_remove_link_flags(bin/trexdebug "-O3 -DEUROPA_FAST")
rospack_remove_link_flags(bin/trexdebug "-lUtils_o -lTREX_o")
rospack_add_link_flags(bin/trexdebug "-lUtils_g -lTREX_g")
target_link_libraries(bin/trexdebug trex_pr2)


# trexfast is about an order of magnitude faster than trexdebug
rospack_add_executable(bin/trexfast src/main.cpp)
rospack_link_boost(bin/trexfast thread)
rospack_add_gtest_build_flags(bin/trexfast)
target_link_libraries(bin/trexfast trex_pr2)

# rospack_add_gtest_build_flags excludes the target from all, on the
# assumption that it's only used as a unit test.  In this case that's not
# true
set_target_properties(bin/trexfast PROPERTIES EXCLUDE_FROM_ALL false)



# Test target for unit tests on topological map integration components
rospack_add_gtest(test/topological_map_tests test/topological_map_tests.cpp)
target_link_libraries(test/topological_map_tests trex_pr2)

# stub_ros__container implements stub state publishers and actions to allow easy testing of the ROS adapters for the model
rospack_add_executable(bin/stub_ros_container src/stub_ros_container.cpp)

# Try the API for running the service calls to the executive
rospack_add_executable(test/ros_reactor_test test/ros_reactor_test.cpp)

