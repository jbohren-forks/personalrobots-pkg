cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(prosilica_cam)

# TODO: figure out how to handle Prosilica SDK dependency
rospack_add_boost_directories()
include_directories($ENV{PROSILICA_SDK}/inc-pc)
link_directories($ENV{PROSILICA_SDK}/bin-pc/x86)

rospack_add_library(prosilica prosilica.cpp)
target_link_libraries(prosilica PvAPI)

rospack_add_executable(prosilica_node prosilica_node.cpp)
target_link_libraries(prosilica_node prosilica)
target_link_libraries(prosilica_node PvAPI)
