cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(ompl_planning)

set(ROS_BUILD_TYPE Debug)

rospack_add_boost_directories()

rospack_add_library(ompl_planning_helpers src/helpers/ompl_planner/PlannerSetup.cpp
                                          src/helpers/ompl_planner/kinematicESTSetup.cpp
                                          src/helpers/ompl_planner/kinematicIKSBLSetup.cpp
                                          src/helpers/ompl_planner/kinematicLazyRRTSetup.cpp
                                          src/helpers/ompl_planner/kinematicRRTSetup.cpp
                                          src/helpers/ompl_planner/kinematicKPIECESetup.cpp
                                          src/helpers/ompl_planner/kinematicLBKPIECESetup.cpp
                                          src/helpers/ompl_planner/kinematicSBLSetup.cpp
                                          src/helpers/ompl_planner/dynamicRRTSetup.cpp
                                          src/helpers/ompl_planner/dynamicKPIECESetup.cpp
                                          src/helpers/ompl_extensions/GoalDefinitions.cpp
                                          src/helpers/ompl_extensions/SpaceInformation.cpp
                                          src/helpers/ompl_extensions/ForwardPropagationModels.cpp
					  src/helpers/RequestHandler.cpp
					  src/helpers/Model.cpp
					  )

rospack_link_boost(ompl_planning_helpers thread)

rospack_add_executable(motion_planner src/motion_planner.cpp)
rospack_add_executable(state_validity_monitor src/state_validity_monitor.cpp)
rospack_add_executable(display_planner_collision_model src/display_planner_collision_model.cpp)

rospack_link_boost(motion_planner thread)
rospack_link_boost(state_validity_monitor thread)
rospack_link_boost(display_planner_collision_model thread)

target_link_libraries(motion_planner ompl_planning_helpers)
target_link_libraries(state_validity_monitor ompl_planning_helpers)
target_link_libraries(display_planner_collision_model ompl_planning_helpers)

