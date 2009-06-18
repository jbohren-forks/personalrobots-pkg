cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(planning_models)
rospack_add_boost_directories()

set(ROS_BUILD_TYPE Debug)

rospack_add_library(planning_models src/kinematic.cpp
				    src/kinematic_state_params.cpp
                                    src/load_mesh.cpp
				    src/output.cpp)

# Unit tests
rospack_add_gtest(test_kinematic test/test_kinematic.cpp)
target_link_libraries(test_kinematic planning_models)
rospack_link_boost(test_kinematic thread)
