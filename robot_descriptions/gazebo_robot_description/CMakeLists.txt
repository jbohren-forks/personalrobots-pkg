cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(gazebo_robot_description)

# find needed paths
find_ros_package(pr2_defs)
find_ros_package(gazebo_robot_description)
find_ros_package(ogre)

# build the ogre mesh files from *.stl (and *.stlb from convex decomposition)
file(GLOB pr2_stl_files ${pr2_defs_PACKAGE_PATH}/meshes/*.stl ${pr2_defs_PACKAGE_PATH}/meshes/convex/*.stlb)
set(pr2_gen_files "")

set(pr2_out_path ${CMAKE_CURRENT_SOURCE_DIR}/Media/models/pr2)

MAKE_DIRECTORY(${pr2_out_path})

find_path(ogre_mesh_upgrade NAMES OgreMeshUpgrade PATHS ${ogre_PACKAGE_PATH}/ogre/bin/)

foreach(it ${pr2_stl_files})
  get_filename_component(basename ${it} NAME_WE)

  # convert to ogre files
  if (ogre_mesh_upgrade)
    add_custom_command(
      OUTPUT ${pr2_out_path}/${basename}.mesh
      COMMAND rosrun 
      ARGS ogre_tools stl_to_mesh ${it} ${pr2_out_path}/${basename}.mesh
      COMMAND ${ogre_mesh_upgrade}/OgreMeshUpgrade
      ARGS ${pr2_out_path}/${basename}.mesh
      DEPENDS ${it})
  else (ogre_mesh_upgrade)
    add_custom_command(
      OUTPUT ${pr2_out_path}/${basename}.mesh
      COMMAND rosrun 
      ARGS ogre_tools stl_to_mesh ${it} ${pr2_out_path}/${basename}.mesh
      DEPENDS ${it})
  endif (ogre_mesh_upgrade)
  
  set(pr2_gen_files ${pr2_gen_files} ${pr2_out_path}/${basename}.mesh)
endforeach(it)

add_custom_target(media_files ALL DEPENDS ${pr2_gen_files})


