cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(stereo_blob_tracker)
genmsg()
#find_library(curl REQUIRED)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(stereo_blob_tracker src/stereo_blob_tracker.cpp src/blob_tracker.cpp)
rospack_add_executable(axis_blob_tracker src/axis_blob_tracker.cpp src/blob_tracker.cpp)
rospack_add_executable(stereo_blob_tracker_listener src/listener.cpp)
rospack_add_executable(blob_tracker_gui src/blob_tracker_gui.cpp)


# bin/stereo_blob_tracker images:=videre/images calparams:=videre/cal_params points:=videre/tracked_points

# bin/stereo_blob_listener points:=videre/tracked_points
