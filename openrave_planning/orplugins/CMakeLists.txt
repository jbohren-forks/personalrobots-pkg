cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(orplugins)

find_ros_package(orrosplanning)

file(GLOB orrosplanning_files ${orrosplanning_PACKAGE_PATH}/lib/*)

set(openrave_plugins )
foreach(it ${orrosplanning_files})
get_filename_component(basename ${it} NAME)
add_custom_command(
  OUTPUT ${CMAKE_SOURCE_DIR}/${basename}
  COMMAND ${CMAKE_COMMAND} -E create_symlink
  ARGS ${it} ${CMAKE_SOURCE_DIR}/${basename}
  DEPENDS ${it})

  set(openrave_plugins ${openrave_plugins} ${CMAKE_SOURCE_DIR}/${basename})
endforeach(it)

add_custom_target(orplugins ALL DEPENDS ${openrave_plugins})
