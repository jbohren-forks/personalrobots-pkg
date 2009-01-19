
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(pr2_defs)

rospack_add_gtest(test_update test/test_update.cpp)

find_ros_package(pr2_defs)

file(GLOB def_files ${CMAKE_CURRENT_SOURCE_DIR}/defs/*.xml)

# xacro file generation
find_ros_package(xacro)
find_ros_package(ivcon)
find_ros_package(convex_decomposition)


#iv files
# iterate through all the stl files and generate *.mesh and *.iv files
file(GLOB pr2_stl_files ${CMAKE_CURRENT_SOURCE_DIR}/meshes/*.stl)
set(pr2_gen_files "")

foreach(it ${pr2_stl_files})
get_filename_component(basepath ${it} PATH)
get_filename_component(basename ${it} NAME_WE)

IF ( ${basename} MATCHES "_convex" )

  message("ignoring stale .._convex.stl file:",${basename})

ELSE ( ${basename} MATCHES "_convex" )

#convex decomposition obj files
add_custom_command(
  OUTPUT ${basepath}/convex
  # CMake 2.4 doesn't offer the make_directory command.
  #COMMAND ${CMAKE_COMMAND} -E make_directory
  COMMAND mkdir -p
  ARGS ${basepath}/convex)

#create obj files for convex decomposition from stl files
add_custom_command(
  OUTPUT ${basepath}/convex/${basename}.obj
  COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
  ARGS ${it} ${basepath}/convex/${basename}.obj
  DEPENDS ${it} ${basepath}/convex)

set(pr2_gen_files ${pr2_gen_files} ${basepath}/convex/${basename}.obj)

add_custom_command(
  OUTPUT ${basepath}/convex/${basename}_convex.obj
  COMMAND ${convex_decomposition_PACKAGE_PATH}/convex_decomposition/bin/convex_decomposition
  ARGS ${basepath}/convex/${basename}.obj -v12 -p10
  DEPENDS ${basepath}/convex/${basename}.obj ${basepath}/convex)

set(pr2_gen_files ${pr2_gen_files} ${basepath}/convex/${basename}_convex.obj)

#convert obj files back to stlb, put in directory named convex
add_custom_command(
  OUTPUT ${basepath}/convex/${basename}_convex.stlb
  COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
  ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.stlb
  DEPENDS ${it} ${basepath}/convex ${basepath}/convex/${basename}_convex.obj)

set(pr2_gen_files ${pr2_gen_files} ${basepath}/convex/${basename}_convex.stlb)

#iv files
add_custom_command(
  OUTPUT ${basepath}/${basename}.iv
  COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
  ARGS ${it} ${basepath}/${basename}.iv
  DEPENDS ${it})
add_custom_command(
  OUTPUT ${basepath}/convex/${basename}_convex.iv
  COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
  ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.iv
  DEPENDS ${it} ${basepath}/convex ${basepath}/convex/${basename}_convex.obj)
 
set(pr2_gen_files ${pr2_gen_files} ${basepath}/${basename}.iv ${basepath}/convex/${basename}_convex.iv)


  ENDIF ( ${basename} MATCHES "_convex" )

endforeach(it)

add_custom_target(media_files ALL DEPENDS ${pr2robot} ${pr2prototype1} ${pr2larm} ${pr2rarm} ${pr2gripper} ${pr2_gen_files})

#TODO; stop bin and lib from being generated
#TODO: delete following section if expanded.xml's are not needed
set(pr2robot "${CMAKE_CURRENT_SOURCE_DIR}/robots/pr2.xacro.xml")
set(pr2robot_expanded "${CMAKE_CURRENT_SOURCE_DIR}/robots/pr2.expanded.xml")
add_custom_command(
  OUTPUT ${pr2robot_expanded}
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2robot} > ${pr2robot_expanded}
  DEPENDS ${pr2robot} ${def_files})

# Replace the COMMAND block with a call to a version of xacro that reads in
# a .deps file and a xacro.xml file, determines the dependencies of the
# latter, and writes any changed dependencies to the .deps file.
#add_custom_target(pr2prototype1_target ALL
#  COMMAND touch ${pr2prototype1}.xacro.deps)

set(pr2prototype1 "${CMAKE_CURRENT_SOURCE_DIR}/robots/prototype1.xacro.xml")
set(pr2prototype1_expanded "${CMAKE_CURRENT_SOURCE_DIR}/robots/prototype1.expanded.xml")
add_custom_command(
  OUTPUT ${pr2prototype1_expanded}
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2prototype1} > ${pr2prototype1_expanded}
  DEPENDS ${pr2prototype1} ${def_files})

set(pr2larm "${CMAKE_CURRENT_SOURCE_DIR}/robots/l_arm.xacro.xml")
set(pr2larm_expanded "${CMAKE_CURRENT_SOURCE_DIR}/robots/l_arm.expanded.xml")
add_custom_command(
  OUTPUT ${pr2larm_expanded}
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2larm} > ${pr2larm_expanded}
  DEPENDS ${pr2larm} ${def_files})

set(pr2rarm "${CMAKE_CURRENT_SOURCE_DIR}/robots/r_arm.xacro.xml")
set(pr2rarm_expanded "${CMAKE_CURRENT_SOURCE_DIR}/robots/r_arm.expanded.xml")
add_custom_command(
  OUTPUT ${pr2rarm_expanded}
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2rarm} > ${pr2rarm_expanded}
  DEPENDS ${pr2rarm} ${def_files})

set(pr2gripper "${CMAKE_CURRENT_SOURCE_DIR}/robots/gripper.xacro.xml")
set(pr2gripper_expanded "${CMAKE_CURRENT_SOURCE_DIR}/robots/gripper.expanded.xml")
add_custom_command(
  OUTPUT ${pr2gripper_expanded}
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2gripper} > ${pr2gripper_expanded}
  DEPENDS ${pr2gripper} ${def_files})


