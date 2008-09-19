cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(control_toolbox)
genmsg()
gensrv()
rospack_add_library(control_toolbox
 src/pid.cpp
 src/sine_sweep.cpp
 )
