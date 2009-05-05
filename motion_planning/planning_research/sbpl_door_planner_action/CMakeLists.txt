cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(sbpl_door_planner_action)
gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
rospack_add_library(sbpl_door_planner_action 
src/sbpl_door_planner_action.cpp)

rospack_add_executable(door_planner_action src/action_runner_sbpl_door_planner.cpp)
target_link_libraries(door_planner_action sbpl_door_planner_action)
