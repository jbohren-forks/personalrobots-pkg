cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(prosilica_cam)
gensrv()

rospack_add_boost_directories()

rospack_add_library(prosilica prosilica.cpp)

rospack_add_executable(prosilica_node prosilica_node.cpp)
target_link_libraries(prosilica_node prosilica)

rospack_add_executable(prosilica_view prosilica_view.cpp)
rospack_add_executable(prosilica_grab prosilica_grab.cpp)
rospack_add_executable(write_memory write_memory.cpp)
target_link_libraries(write_memory prosilica)
#rospack_add_executable(ListCameras ListCameras.cpp)
#rospack_add_executable(Ping Ping.cpp)
#rospack_add_executable(CLIpConfig CLIpConfig.cpp)
rospack_add_executable(calibration calibration.cpp cvcalibinit.cpp)
rospack_add_executable(yml2ini yml2ini.cpp)
