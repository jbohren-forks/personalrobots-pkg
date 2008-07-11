cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wg_collision_space)
include_directories(${wg_collision_space_INCLUDE_DIRS})
rospack_add_library(CollisionSpace src/collisionspace/kinematicODE.cpp
				   src/collisionspace/kinematicSOLID.cpp
				   src/collisionspace/environmentODE.cpp
				   src/collisionspace/environmentSOLID.cpp)
link_directories(${wg_collision_space_LIBRARY_DIRS})
rospack_add_executable(test_kinematic_ode src/test/test_kinematic_ode.cpp)
