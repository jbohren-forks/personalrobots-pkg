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
                                          src/helpers/ompl_planner/kinematicpRRTSetup.cpp
                                          src/helpers/ompl_planner/kinematicKPIECESetup.cpp
                                          src/helpers/ompl_planner/kinematicLBKPIECESetup.cpp
                                          src/helpers/ompl_planner/kinematicSBLSetup.cpp
                                          src/helpers/ompl_planner/kinematicpSBLSetup.cpp
                                          src/helpers/ompl_planner/dynamicRRTSetup.cpp
                                          src/helpers/ompl_planner/dynamicKPIECESetup.cpp
					  src/helpers/ompl_planner/IKSetup.cpp
                                          src/helpers/ompl_extensions/GoalDefinitions.cpp
                                          src/helpers/ompl_extensions/SpaceInformation.cpp
                                          src/helpers/ompl_extensions/StateValidator.cpp
                                          src/helpers/ompl_extensions/ForwardPropagationModels.cpp
					  src/helpers/Model.cpp
					  src/helpers/ModelBase.cpp
					  )
rospack_link_boost(ompl_planning_helpers thread)

rospack_add_executable(motion_planner src/motion_planner.cpp src/request_handler/RequestHandler.cpp)
rospack_link_boost(motion_planner thread)
target_link_libraries(motion_planner ompl_planning_helpers)
