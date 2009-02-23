cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(sbpl_arm_planner)
gensrv()

include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(sbpl_arm_planner
		              src/discrete_space_information/robarm3d/environment_robarm3d.cpp
		              src/utils/utils.cpp
                   )

# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(planPath src/plan_path.cpp
                                src/discrete_space_information/robarm3d/environment_robarm3d.cpp
                                src/utils/utils.cpp)

target_link_libraries(planPath sbpl_arm_planner)


# rospack_add_compile_flags(plan_path_stats "-g")
# rospack_add_link_flags(plan_path )
