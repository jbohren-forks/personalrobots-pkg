cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(prosilica_cam)
gensrv()

# TODO: figure out how to handle Prosilica SDK dependency
rospack_add_boost_directories()
#include_directories($ENV{PROSILICA_SDK}/include)
#link_directories($ENV{PROSILICA_SDK}/lib)

rospack_add_library(prosilica prosilica.cpp)
#target_link_libraries(prosilica PvAPI)

rospack_add_executable(prosilica_node prosilica_node.cpp)
target_link_libraries(prosilica_node prosilica)

rospack_add_executable(prosilica_view prosilica_view.cpp)
rospack_add_executable(prosilica_grab prosilica_grab.cpp)
