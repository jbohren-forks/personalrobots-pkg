cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(wg_robot_description)

rospack_add_gtest(test_update test/test_update.cpp)

rospack_add_executable(ivcon src/ivcon)

find_ros_package(wg_robot_description)

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
  ARGS < ${it} > ${basepath}/${basename}.iv
  DEPENDS ${it})

#ogre files
add_custom_command(
  OUTPUT ${basepath}/${basename}.mesh
  COMMAND rosrun 
  ARGS ogre_tools stl_to_mesh ${it} ${basepath}/${basename}.mesh
  DEPENDS ${it})

set(pr2_gen_files ${pr2_gen_files} ${basepath}/${basename}.iv ${basepath}/${basename}.mesh)

endforeach(it)

add_custom_target(media_files ALL DEPENDS ${pr2_gen_files})
