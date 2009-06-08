cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(planning_environment)

set(ROS_BUILD_TYPE Release)

rospack_add_library(planning_environment src/robot_models.cpp
					 src/collision_models.cpp
					 src/kinematic_model_state_monitor.cpp
					 src/collision_space_monitor.cpp)

					 
# Tests

# Create a model of the PR2
rospack_add_executable(test_robot_models test/test_robot_models.cpp)
rospack_declare_test(test_robot_models)
rospack_add_gtest_build_flags(test_robot_models)
target_link_libraries(test_robot_models planning_environment)
rospack_add_rostest(test/test_robot_models.xml)
