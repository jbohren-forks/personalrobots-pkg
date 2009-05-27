cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rospack(cv_mech_turk)
genmsg()

rospack_add_executable(cv_mturk src/cv_mturk.cpp)
rospack_add_executable(mt_manual_select src/mt_manual_select.cpp)
rospack_add_executable(mt_manual_select_cv src/mt_manual_select_cv.cpp)
rospack_add_executable(convert_session_results_to_mask src/convert_session_results_to_mask.cpp)

