cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(prosilica_gige_sdk)

link_directories($ENV{PWD}/../lib)
link_libraries(PvAPI)
add_definitions(-D_LINUX -D_x86)
set(EXAMPLES build/Prosilica_GigE_SDK/examples)

rospack_add_executable(bin/CLIpConfig ${EXAMPLES}/CLIpConfig/CLIpConfig.cpp)
rospack_add_executable(bin/Ping ${EXAMPLES}/Ping/Ping.cpp)
rospack_add_executable(bin/ListCameras ${EXAMPLES}/ListCameras/ListCameras.cpp)
rospack_add_executable(bin/ResetCamera ${EXAMPLES}/ResetCamera/ResetCamera.cpp)
#TODO: SampleViewer
