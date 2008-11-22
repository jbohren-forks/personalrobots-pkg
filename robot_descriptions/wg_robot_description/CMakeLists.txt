cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wg_robot_description)

rospack_add_gtest(test_update test/test_update.cpp)
rospack_add_executable(ivcon src/ivcon)

find_ros_package(wg_robot_description)

# xacro file generation
find_ros_package(xacro)

set(pr2robot "${CMAKE_CURRENT_SOURCE_DIR}/pr2/pr2")
add_custom_command(
  OUTPUT ${pr2robot}.xml
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2robot}.xacro.xml > ${pr2robot}.xml
  DEPENDS ${pr2robot}.xacro.xml)


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
  DEPENDS ${pr2prototype1}.xacro.xml)

set(pr2arm "${CMAKE_CURRENT_SOURCE_DIR}/pr2_arm_test/pr2_arm")
add_custom_command(
  OUTPUT ${pr2arm}.xml
  COMMAND ${xacro_PACKAGE_PATH}/xacro.py
  ARGS ${pr2arm}.xacro.xml > ${pr2arm}.xml
  DEPENDS ${pr2arm}.xacro.xml)

#iv files
# iterate through all the stl files and generate *.mesh and *.iv files
file(GLOB pr2_stl_files ${CMAKE_CURRENT_SOURCE_DIR}/models/pr2/*.stl)
set(pr2_gen_files "")

foreach(it ${pr2_stl_files})
get_filename_component(basepath ${it} PATH)
get_filename_component(basename ${it} NAME_WE)

#iv files
add_custom_command(
  OUTPUT ${basepath}/${basename}.iv
  COMMAND ivcon
  ARGS ${it} ${basepath}/${basename}.iv
  DEPENDS ${it} ivcon)

set(pr2_gen_files ${pr2_gen_files} ${basepath}/${basename}.iv)

endforeach(it)

add_custom_target(media_files ALL DEPENDS ${pr2robot}.xml ${pr2prototype1}.xml ${pr2arm}.xml ${pr2_gen_files})
