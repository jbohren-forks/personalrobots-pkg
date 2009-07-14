cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(planning_environment)

set(ROS_BUILD_TYPE Debug)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rospack_add_library(planning_environment src/models/robot_models.cpp
					 src/models/collision_models.cpp
					 src/monitors/kinematic_model_state_monitor.cpp
					 src/monitors/collision_space_monitor.cpp
					 src/monitors/planning_monitor.cpp
					 src/util/kinematic_state_constraint_evaluator.cpp
					 src/util/construct_object.cpp)
rospack_add_openmp_flags(planning_environment)
rospack_link_boost(planning_environment thread)
 
# Utility apps
rospack_add_executable(view_state_validity src/tools/view_state_validity.cpp) 
target_link_libraries(view_state_validity planning_environment)
rospack_link_boost(view_state_validity thread)
rospack_add_openmp_flags(view_state_validity)

rospack_add_executable(clear_known_objects src/tools/clear_known_objects.cpp) 
target_link_libraries(clear_known_objects planning_environment)
rospack_link_boost(clear_known_objects thread)
rospack_add_openmp_flags(clear_known_objects)

rospack_add_executable(display_planner_collision_model src/tools/display_planner_collision_model.cpp) 
target_link_libraries(display_planner_collision_model planning_environment)
rospack_link_boost(display_planner_collision_model thread)
rospack_add_openmp_flags(display_planner_collision_model)

# Tests

# Create a model of the PR2
rospack_add_executable(test_robot_models test/test_robot_models.cpp) 
rospack_declare_test(test_robot_models)
rospack_add_gtest_build_flags(test_robot_models)
target_link_libraries(test_robot_models planning_environment)
rospack_add_rostest(test/test_robot_models.xml)
