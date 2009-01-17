cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(cv_wrappers)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# cv_view* (note: does not include cv_view in service/)
rospack_add_executable(cv_view src/cv_view.cpp)
rospack_add_executable(cv_view_minimal minimal/cv_view.cpp)
rospack_add_executable(cv_view_array src/cv_view_array.cpp)
rospack_add_executable(cv_view_dcam dcam/cv_view.cpp)
rospack_add_executable(cv_view_array_overlay src/cv_view_array_overlay.cpp)

# cv_movie_streamer
rospack_add_executable(cv_movie_streamer src/cv_movie_streamer.cpp)

# cv_cam_calib
rospack_add_executable(cv_cam_calib src/cv_cam_calib.cpp)
