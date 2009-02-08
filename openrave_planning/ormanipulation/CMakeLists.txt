cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
add_definitions(-Wall)
rospack(ormanipulation)

# process all urdf files
set(GRASP_TABLE_OUTPUT ${CMAKE_SOURCE_DIR}/octave/grasp_pr2_ricebox.mat)

add_custom_command(
  OUTPUT ${GRASP_TABLE_OUTPUT}
  COMMAND roslaunch "${CMAKE_SOURCE_DIR}/maketables_noviewer.ros.xml"
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/octave
  DEPENDS  ${CMAKE_SOURCE_DIR}/octave/runmaketables.m ${CMAKE_SOURCE_DIR}/octave/MakePR2GraspTables.m)
add_custom_target(grasp_tables ALL DEPENDS ${GRASP_TABLE_OUTPUT})
