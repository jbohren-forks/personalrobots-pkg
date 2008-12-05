cmake_minimum_required(VERSION 2.6)
include(rosbuild)

rospack(mech_turk)
genmsg()

rospack_add_executable(bin/cv_mturk src/cv_mturk.cpp)

