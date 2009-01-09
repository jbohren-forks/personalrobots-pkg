cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(libfz_ht_superpixels)
include_directories(src)
rospack_add_library(fz_ht_superpixels 
  src/disjoint-set.cc
  src/segment-graph.cc
  src/segment-image.cc
  src/filter.cc)
