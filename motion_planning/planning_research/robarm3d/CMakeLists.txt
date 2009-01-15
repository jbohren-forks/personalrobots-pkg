cmake_minimum_required(VERSION 2.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(robarm3d)
gensrv()

include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(robarm3d
		      src/discrete_space_information/robarm/environment_robarm3d.cpp
		      src/utils/utils.cpp
		    )

# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# # rospack_add_executable(plan_path_stats src/test/plan_path_stats.cpp)
# rospack_add_executable(plan_path_stats src/test/test_path_planner.cpp)

rospack_add_executable(planPath src/test/plan_path.cpp)
target_link_libraries(planPath robarm3d)

rospack_add_executable(planPathROS src/test/plan_path_ROS.cpp)
target_link_libraries(planPathROS robarm3d)

rospack_add_executable(sim_path src/test/sim_path_srv.cpp)
target_link_libraries(sim_path robarm3d)

rospack_add_executable(planPathNode src/test/plan_path_node.cpp)
target_link_libraries(planPathNode robarm3d)

# rospack_add_compile_flags(plan_path_stats "-g")
# rospack_add_link_flags(plan_path )
