cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(wg_robot_description)

rospack_add_gtest(test_update test/test_update.cpp)

find_ros_package(wg_robot_description)

file(GLOB def_files ${CMAKE_CURRENT_SOURCE_DIR}/pr2_robot_defs/*.xml)

# xacro file generation
find_ros_package(xacro)
find_ros_package(ivcon)
find_ros_package(convex_decomposition)

set(pr2robot "${CMAKE_CURRENT_SOURCE_DIR}/pr2/pr2")
add_custom_command(
  OUTPUT ${pr2robot}.xml
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2robot}.xacro.xml > ${pr2robot}.xml
  DEPENDS ${pr2robot}.xacro.xml ${def_files})

# Replace the COMMAND block with a call to a version of xacro that reads in
# a .deps file and a xacro.xml file, determines the dependencies of the
# latter, and writes any changed dependencies to the .deps file.
#add_custom_target(pr2prototype1_target ALL
#  COMMAND touch ${pr2prototype1}.xacro.deps)

set(pr2prototype1 "${CMAKE_CURRENT_SOURCE_DIR}/pr2_prototype1/pr2_prototype1")

add_custom_command(
  OUTPUT ${pr2prototype1}.xml
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2prototype1}.xacro.xml > ${pr2prototype1}.xml
  DEPENDS ${pr2prototype1}.xacro.xml ${def_files})

set(pr2larm "${CMAKE_CURRENT_SOURCE_DIR}/pr2_arm_test/pr2_l_arm")
add_custom_command(
  OUTPUT ${pr2larm}.xml
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2larm}.xacro.xml > ${pr2larm}.xml
  DEPENDS ${pr2larm}.xacro.xml ${def_files})

set(pr2rarm "${CMAKE_CURRENT_SOURCE_DIR}/pr2_arm_test/pr2_r_arm")
add_custom_command(
  OUTPUT ${pr2rarm}.xml
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2rarm}.xacro.xml > ${pr2rarm}.xml
  DEPENDS ${pr2rarm}.xacro.xml ${def_files})

set(pr2gripper "${CMAKE_CURRENT_SOURCE_DIR}/pr2_gripper/pr2_gripper")
add_custom_command(
  OUTPUT ${pr2gripper}.xml
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2gripper}.xacro.xml > ${pr2gripper}.xml
  DEPENDS ${pr2gripper}.xacro.xml ${def_files})

#iv files
# iterate through all the stl files and generate *.mesh and *.iv files
file(GLOB pr2_stl_files ${CMAKE_CURRENT_SOURCE_DIR}/models/pr2/*.stl)
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

add_custom_target(media_files ALL DEPENDS ${pr2robot}.xml ${pr2prototype1}.xml ${pr2larm}.xml ${pr2rarm}.xml ${pr2gripper}.xml ${pr2_gen_files})
