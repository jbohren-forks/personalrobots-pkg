cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(or_plugins)

find_ros_package(or_rosplanning)

file(GLOB or_rosplanning_files ${or_rosplanning_PACKAGE_PATH}/lib/*)

set(openrave_plugins )
foreach(it ${or_rosplanning_files})
get_filename_component(basename ${it} NAME)
add_custom_command(
  OUTPUT ${CMAKE_SOURCE_DIR}/${basename}
  COMMAND ${CMAKE_COMMAND} -E create_symlink
  ARGS ${it} ${CMAKE_SOURCE_DIR}/${basename}
  DEPENDS ${it})

  set(openrave_plugins ${openrave_plugins} ${CMAKE_SOURCE_DIR}/${basename})
endforeach(it)

add_custom_target(or_plugins ALL DEPENDS ${openrave_plugins})
