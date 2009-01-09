cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(robarm3d)

include_directories(${PROJECT_SOURCE_DIR}/code)
rospack_add_library(robarm3d
		      src/discrete_space_information/robarm/environment_robarm3d.cpp
		      src/utils/utils.cpp
		    )

# Build executables in the bin directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(plan_path src/test/test.cpp)
rospack_add_executable(test_forward_kinematics src/test/test_forward_kinematics.cpp)
# rospack_add_executable(sim_path_gazebo src/test/simulate_arm_path.cpp)
target_link_libraries(plan_path robarm3d)
rospack_add_compile_flags(plan_path "-g -pg")
rospack_add_link_flags(plan_path "-g -pg")
