cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(point_cloud_assembler)
gensrv()

rospack_add_boost_directories()

rospack_add_executable(laser_scan_assembler_srv src/laser_scan_assembler_srv.cpp)
rospack_add_executable(point_cloud_assembler_srv src/point_cloud_assembler_srv.cpp)
rospack_add_executable(grab_cloud_data src/grab_cloud_data.cpp)
rospack_add_executable(point_cloud_snapshotter src/point_cloud_snapshotter.cpp)
rospack_add_executable(point_cloud_srv src/point_cloud_srv.cpp)
rospack_add_executable(merge_clouds src/merge_clouds.cpp)
rospack_link_boost(merge_clouds thread)

# Needed to run rostests
rospack_add_executable(dummy_scan_producer test/dummy_scan_producer.cpp)

rospack_add_executable(test_assembler 
                       test/test_assembler.cpp)
rospack_link_boost(test_assembler thread)

rospack_add_rostest(${CMAKE_CURRENT_SOURCE_DIR}/test/test_laser_assembler.launch)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
