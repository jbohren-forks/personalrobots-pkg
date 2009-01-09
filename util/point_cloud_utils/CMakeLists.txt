cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(point_cloud_utils)

rospack_add_library(point_cloud_utils src/blob_finder.cpp
                                      src/rigid_transform_finder.cpp
                                      src/scan_assembler.cpp
                                      src/timed_scan_assembler.cpp)
                                      
#rospack_add_executable(timed_scan_tester src/timed_scan_tester.cpp)
#target_link_libraries(timed_scan_tester point_cloud_utils)