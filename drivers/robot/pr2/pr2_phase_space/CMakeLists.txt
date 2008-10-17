cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(pr2_phase_space)
genmsg()
rospack_add_executable(phase_space_odometry src/phase_space_odometry.cpp)
rospack_add_executable(phase_space_pan_tilt src/phase_space_pan_tilt.cpp)
rospack_add_executable(phase_space_arm src/phase_space_arm.cpp)
rospack_add_executable(phase_space_fake_localization src/phase_space_fake_localization.cpp)


