cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(3dmgx2_driver)
rospack_add_library(3dmgx2 3dmgx2.cc)
