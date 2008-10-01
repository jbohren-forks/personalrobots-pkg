cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(spacenav_node)
rospack_add_executable( spacenav_node
  src/spacenav_node.cpp
  )
