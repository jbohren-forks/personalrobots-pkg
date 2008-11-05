cmake_minimum_required(VERSION 2.6)
include(rosbuild)

set(ROS_BUILD_TYPE Release)

rospack(dcam)

rospack_add_library(dcam src/dcam.cpp src/stereocam.cpp src/image.cpp src/stereolib.c)

rospack_add_compile_flags(dcam "-msse2 -mpreferred-stack-boundary=4")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

rospack_add_executable(stereocal src/stereocal.cpp src/stereogui.cpp src/imwin.cpp src/calwin.cpp)
target_link_libraries(stereocal dcam fltk)

rospack_add_executable(acquire src/acquire.cpp)
target_link_libraries(acquire dcam fltk)

rospack_add_executable(stacquire src/stacquire.cpp src/imwin.cpp)
target_link_libraries(stacquire dcam fltk)