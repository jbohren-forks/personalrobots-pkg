cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(display_ode_spaces)

rospack_add_library(display_ODE_space src/displayODE.cpp)
