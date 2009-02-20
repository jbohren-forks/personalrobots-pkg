cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(kinematic_planning)

rospack_add_boost_directories()

rospack_add_library(kinematic_planning_helpers src/helpers/ompl_planner/RKPPlannerSetup.cpp
                                               src/helpers/ompl_planner/RKPESTSetup.cpp
                                               src/helpers/ompl_planner/RKPIKSBLSetup.cpp
                                               src/helpers/ompl_planner/RKPLazyRRTSetup.cpp
                                               src/helpers/ompl_planner/RKPRRTSetup.cpp
                                               src/helpers/ompl_planner/RKPIKKPIECESetup.cpp
                                               src/helpers/ompl_planner/RKPKPIECESetup.cpp
                                               src/helpers/ompl_planner/RKPSBLSetup.cpp
					       )

rospack_add_executable(kinematic_planning src/kinematic_planning.cpp)
rospack_add_executable(motion_validator src/motion_validator.cpp)
rospack_add_executable(state_validity_monitor src/state_validity_monitor.cpp)
rospack_add_executable(display_planner_collision_model src/display_planner_collision_model.cpp)

rospack_link_boost(kinematic_planning thread)
rospack_link_boost(motion_validator thread)
rospack_link_boost(state_validity_monitor thread)
rospack_link_boost(display_planner_collision_model thread)

target_link_libraries(kinematic_planning kinematic_planning_helpers)
target_link_libraries(motion_validator kinematic_planning_helpers)
target_link_libraries(state_validity_monitor kinematic_planning_helpers)
target_link_libraries(display_planner_collision_model kinematic_planning_helpers)

add_subdirectory(test)
