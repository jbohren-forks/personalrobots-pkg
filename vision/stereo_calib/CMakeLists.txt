cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(stereo_calib)
#genmsg()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(stereocal src/stereocal.cpp src/stereogui.cpp src/imwin.cpp src/calwin.cpp src/stereolib.c src/cloud_transform.cpp)

rospack_add_executable(stlrf src/stlrf.cpp src/stereolrfgui.cpp src/imwin.cpp src/calwin.cpp src/cloud_transform.cpp)

target_link_libraries(stereocal fltk)
target_link_libraries(stlrf fltk)


# bin/stereo_blob_tracker images:=videre/images calparams:=videre/cal_params points:=videre/tracked_points

# bin/stereo_blob_listener points:=videre/tracked_points
