cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(libfz_ht_superpixels)
include_directories(src)
rospack_add_library(fz_ht_superpixels 
  src/disjoint-set.cc
  src/segment-graph.cc
  src/segment-image.cc
  src/filter.cc)
