cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(point_cloud_assembler)
gensrv()

rospack_add_executable(point_cloud_assembler src/point_cloud_assembler.cpp)
rospack_add_executable(grab_cloud_data src/grab_cloud_data.cpp)
rospack_add_executable(point_cloud_snapshotter src/point_cloud_snapshotter.cpp)

