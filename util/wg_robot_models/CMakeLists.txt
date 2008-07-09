cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wg_robot_models)
include_directories(${wg_robot_models_INCLUDE_DIRS})
rospack_add_library(RobotModels src/robotmodels/kinematic.cpp
				src/robotmodels/kinematicODE.cpp
				src/robotmodels/kinematicSOLID.cpp)
link_directories(${wg_robot_models_LIBRARY_DIRS})
rospack_add_executable(test_kinematic_ode src/test/test_kinematic_ode.cpp)
