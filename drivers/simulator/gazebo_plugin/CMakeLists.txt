cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(gazebo_plugin)
rospack_add_library(Pr2_Actarray src/Pr2_Actarray.cc)
rospack_add_library(Pr2_Gripper src/Pr2_Gripper.cc)
rospack_add_library(Ros_Camera src/Ros_Camera.cc)
rospack_add_library(Ros_Node src/Ros_Node.cc)
rospack_add_library(P3D src/P3D.cc)

# This target requires adding player to the manifest.xml.
#rospack_add_executable(player_pr2 src/player/Pr2_Player.cc)
#target_link_libraries(player_pr2 playerc++)


# TODO: remove this hardcoded dependency
include_directories(/usr/include/libxml2)

