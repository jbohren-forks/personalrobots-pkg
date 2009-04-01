cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(sbpl_door_planner)
gensrv()

include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(sbpl_door_planner
                    src/discrete_space_information/navxythetadoor/environment_navxythetadoor.cpp
                   )

# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(planDoorPath src/test/main.cpp
                                    src/discrete_space_information/navxythetadoor/environment_navxythetadoor.cpp)

target_link_libraries(planDoorPath sbpl_door_planner)

