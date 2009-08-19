cmake_minimum_required(VERSION 2.4.6)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)

rosbuild_init()
rosbuild_add_boost_directories()

rosbuild_genmsg()
rosbuild_gensrv()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)


add_custom_command(
    OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/build/Ikea_meshes.tar.gz
    COMMAND wget
    ARGS pr.willowgarage.com/downloads/Ikea_meshes.tar.gz
    DEPENDS )

#un tar Ikea mesh files
add_custom_command(
  OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/meshes
  COMMAND mkdir -p
  ARGS ${CMAKE_CURRENT_SOURCE_DIR}/meshes
  COMMAND tar
  ARGS -xvzf ${CMAKE_CURRENT_SOURCE_DIR}/build/Ikea_meshes.tar.gz -C ${CMAKE_CURRENT_SOURCE_DIR}/meshes
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/build/Ikea_meshes.tar.gz)

add_custom_target(media_files ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/meshes)


rosbuild_add_executable(recognition_lambertian src/recognition_lambertian.cpp) 
rosbuild_add_executable (plane_fit src/plane_fit.cpp)
rosbuild_add_executable (publish_scene src/publish_scene.cpp)
rosbuild_add_executable(rec_lam_normal_features src/rec_lam_normal_features.cpp src/visualization.cpp)
rosbuild_add_executable(model_fitter src/model_fitter.cpp src/ply.c)
rosbuild_link_boost(model_fitter filesystem)
