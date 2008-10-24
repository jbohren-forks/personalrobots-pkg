cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(point_cloud_utils)
rospack_add_library(point_cloud_utils src/blob_finder.cpp
                                      src/rigid_transform_finder.cpp)