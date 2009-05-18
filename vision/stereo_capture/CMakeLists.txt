cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(stereo_capture)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(ROS_BUILD_TYPE RelWithDebInfo)

rospack_add_executable(stereo_capture src/stereo_capture.cpp)
#rospack_add_executable(skindetector src/skindetector.cpp src/cvadaptiveskindetector.cpp)
rospack_add_executable(project_light src/project_light.cpp)
rospack_add_executable(FindOutlet src/FindOutlet.cpp)
rospack_add_executable(RectifyOutlets src/RectifyOutlets.cpp src/outlet_tuple.cpp)

rospack_add_executable(periodic_capture src/periodic_capture.cpp)
