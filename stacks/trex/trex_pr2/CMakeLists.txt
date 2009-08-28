cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
rosbuild_init()

rosbuild_genmsg()
rosbuild_gensrv()

rosbuild_add_boost_directories()

find_ros_package(trex_ros)
include(${trex_ros_PACKAGE_PATH}/trex_build.cmake)
create_nddl_config()


set(TREX_FILES src/pr2_adapter_utilities.cpp
	       src/pr2_components.cpp
	       src/topological_map.cpp
	       src/door_domain_constraints.cpp
	       src/pr2_adapters.cpp
	       src/master_reactor.cpp)

# build extensions into exported library
create_trex_lib(trex_pr2 TREX_FILES)


# run the tmap creator
execute_process(COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/test/createTmaps.sh) 

# build executables
create_trex_executables(bin/trexfast bin/trexdebug src/main.cpp)

# Test target for unit tests on topological map integration components
rosbuild_add_gtest(test/topological_map_tests test/topological_map_tests.cpp)
rosbuild_add_gtest_build_flags(test/topological_map_tests)
target_link_libraries(test/topological_map_tests trex_pr2_o)

# stub_ros__container implements stub state publishers and actions to allow easy testing of the ROS adapters for the model
rosbuild_add_executable(bin/stub_ros_container src/stub_ros_container.cpp)

# Try the API for running the service calls to the executive
rosbuild_add_executable(test/ros_reactor_test test/ros_reactor_test.cpp)

# Model and executive tests with NDDL simulator
rosbuild_add_rostest(test/m2_lite/sim/pr2.test)
rosbuild_add_rostest(test/drive_open_drive/sim/pr2.test)

