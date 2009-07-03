cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE Release)
rospack(trex_temp_proj)

genmsg()
gensrv()

rospack_add_boost_directories()

###############################################################################
# NDDL Include Macro
###############################################################################

# This generates the NDDL inclue configuration file
find_ros_package(trex_ros)
include(${trex_ros_PACKAGE_PATH}/NDDL.cmake)
create_nddl_config()

###############################################################################
# TREX Hooks into Local ROS Code
###############################################################################

set(TREX_TEMP_PROJ_FILES src/adapter_utilities.cpp
			 src/components.cpp
			 src/adapters.cpp
			 src/master_reactor.cpp)

###############################################################################
# Executables & Tests
###############################################################################

# trexdebug builds with a large number of run-time error checking running which is expensive
# but gives good feedback in discovering problems.
rospack_add_library(executive_trex_temp_proj_g ${TREX_TEMP_PROJ_FILES})
target_link_libraries(executive_trex_temp_proj_g executive_trex_pr2_g Utils_g TREX_g)
rospack_add_executable(bin/trexdebug src/main.cpp)
target_link_libraries(bin/trexdebug executive_trex_temp_proj_g )
rospack_link_boost(bin/trexdebug thread)
rospack_add_gtest_build_flags(bin/trexdebug)

# trexfast is about an order of magnitude faster than trexdebug
rospack_add_library(executive_trex_temp_proj_o ${TREX_TEMP_PROJ_FILES})
target_link_libraries(executive_trex_temp_proj_o executive_trex_pr2_o Utils_o TREX_o)
rospack_add_executable(bin/trexfast src/main.cpp)
target_link_libraries(bin/trexfast executive_trex_temp_proj_o)
rospack_link_boost(bin/trexfast thread)
rospack_add_gtest_build_flags(bin/trexfast)

# rospack_add_gtest_build_flags excludes the target from all, on the
# assumption that it's only used as a unit test.  In this case that's not
# true
set_target_properties(bin/trexfast PROPERTIES EXCLUDE_FROM_ALL false)
rospack_add_compile_flags(bin/trexfast "-O3 -DEUROPA_FAST")
rospack_add_compile_flags(executive_trex_temp_proj_o "-O3 -DEUROPA_FAST")


# stub_ros__container implements stub state publishers and actions to allow easy testing of the ROS adapters for the model
rospack_add_executable(bin/stub_ros_container src/stub_ros_container.cpp)

# Try the API for running the service calls to the executive
rospack_add_executable(test/ros_reactor_test test/ros_reactor_test.cpp)

