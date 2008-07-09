cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(pr2SimRT)
rospack_add_executable(pr2SimRT pr2SimRT.cc)
