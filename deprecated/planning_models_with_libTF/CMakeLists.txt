cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(planning_models_with_libTF)
rospack_add_library(planning_models_with_libtf src/planning_models/kinematic.cpp
#				    src/planning_models/kinodynamic.cpp
				    )
