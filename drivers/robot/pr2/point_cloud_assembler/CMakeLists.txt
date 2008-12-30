cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(point_cloud_assembler)
gensrv()

rospack_add_executable(laser_scan_assembler_srv src/laser_scan_assembler_srv.cpp)
rospack_add_executable(point_cloud_assembler_srv src/point_cloud_assembler_srv.cpp)
rospack_add_executable(grab_cloud_data src/grab_cloud_data.cpp)
rospack_add_executable(point_cloud_snapshotter src/point_cloud_snapshotter.cpp)

rospack_add_executable(../test/bin/test_assembler 
                       test/test_assembler.cpp)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_laser_assembler.launch)
rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_cloud_assembler.launch)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)