cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(planning_models)
rospack_add_boost_directories()

set(ROS_BUILD_TYPE Release)

rospack_add_library(planning_models src/kinematic.cpp
				    src/kinematic_state_params.cpp
				    src/output.cpp)
rospack_link_boost(planning_models thread)


rospack_add_library(planning_models2 src/kinematic_model.cpp)
rospack_link_boost(planning_models2 thread)

# Unit tests
rospack_add_gtest(test_kinematic test/test_kinematic.cpp)
target_link_libraries(test_kinematic planning_models)
rospack_link_boost(test_kinematic thread)

#rospack_add_gtest(test_kinematic2 test/test_kinematic2.cpp)
#target_link_libraries(test_kinematic2 planning_models2)
#rospack_link_boost(test_kinematic2 thread)
