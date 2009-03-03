cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(ompl_planning)

rospack_add_boost_directories()

rospack_add_library(ompl_planning_helpers src/helpers/ompl_planner/RKPPlannerSetup.cpp
                                          src/helpers/ompl_planner/RKPESTSetup.cpp
                                          src/helpers/ompl_planner/RKPIKSBLSetup.cpp
                                          src/helpers/ompl_planner/RKPLazyRRTSetup.cpp
                                          src/helpers/ompl_planner/RKPRRTSetup.cpp
                                          src/helpers/ompl_planner/RKPIKKPIECESetup.cpp
                                          src/helpers/ompl_planner/RKPKPIECESetup.cpp
                                          src/helpers/ompl_planner/RKPSBLSetup.cpp
					  )

rospack_add_executable(motion_planner src/motion_planner.cpp)
rospack_add_executable(motion_validator src/motion_validator.cpp)
rospack_add_executable(state_validity_monitor src/state_validity_monitor.cpp)
rospack_add_executable(display_planner_collision_model src/display_planner_collision_model.cpp)

rospack_link_boost(motion_planner thread)
rospack_link_boost(motion_validator thread)
rospack_link_boost(state_validity_monitor thread)
rospack_link_boost(display_planner_collision_model thread)

target_link_libraries(motion_planner ompl_planning_helpers)
target_link_libraries(motion_validator ompl_planning_helpers)
target_link_libraries(state_validity_monitor ompl_planning_helpers)
target_link_libraries(display_planner_collision_model ompl_planning_helpers)

add_subdirectory(test)
