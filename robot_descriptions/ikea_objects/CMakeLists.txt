
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(ikea_objects)

# xacro file generation
find_ros_package(ikea_objects)
find_ros_package(xacro)
find_ros_package(ivcon)
find_ros_package(convex_decomposition)

#iv files
# iterate through all the obj files and generate *.mesh and *.iv files
file(GLOB ikea_objects_obj_files ${CMAKE_CURRENT_SOURCE_DIR}/meshes/*.obj)
set(ikea_objects_gen_files "")

foreach(it ${ikea_objects_obj_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME)

  IF ( ${basename} MATCHES "_convex" )
    message("ignoring stale .._convex.* file:",${basename})
  ELSE ( ${basename} MATCHES "_convex" )

  #convex directory
  add_custom_command(
    OUTPUT ${basepath}/convex
    COMMAND mkdir -p
    ARGS ${basepath}/convex)

  #move obj files to convex directory
  add_custom_command(
    OUTPUT ${basepath}/convex/${basename}
    COMMAND cp
    ARGS ${it} ${basepath}/convex/${basename}
    DEPENDS ${it} ${basepath}/convex)

  set(ikea_objects_gen_files ${ikea_objects_gen_files} ${basepath}/convex/${basename})

  #convex decomp in convex directory
  add_custom_command(
    OUTPUT ${basepath}/convex/${basename}_convex.obj
    COMMAND ${convex_decomposition_PACKAGE_PATH}/convex_decomposition/bin/convex_decomposition
    ARGS ${basepath}/convex/${basename} -v64 -p0.01 -c0.01 -d7
    DEPENDS ${basepath}/convex/${basename})

  set(ikea_objects_gen_files ${ikea_objects_gen_files} ${basepath}/convex/${basename}_convex.obj)

  #create convex stl files for convex decomposition obj files
  add_custom_command(
    OUTPUT ${basepath}/convex/${basename}_convex.stlb
    #COMMAND /usr/bin/meshlabserver
    COMMAND ${ivcon_PACKAGE_PATH}/bin/ivcon
    #ARGS -i ${basepath}/convex/${basename}_convex.obj -o ${basepath}/convex/${basename}_convex.stl
    ARGS ${basepath}/convex/${basename}_convex.obj ${basepath}/convex/${basename}_convex.stlb
    DEPENDS ${basepath}/convex/${basename}_convex.obj)

  set(ikea_objects_gen_files ${ikea_objects_gen_files} ${basepath}/convex/${basename}_convex.stlb)


  ENDIF ( ${basename} MATCHES "_convex" )

endforeach(it)

add_custom_target(media_files ALL DEPENDS ${ikea_objects_gen_files})

#TODO; stop bin and lib from being generated
file(GLOB convex_objects_ply_files ${CMAKE_CURRENT_SOURCE_DIR}/meshes/convex/*_convex.obj)
set(convex_objects_gen_files "")

foreach(it ${convex_objects_ply_files})
  get_filename_component(basepath ${it} PATH)
  get_filename_component(basename ${it} NAME)

  #create convex stl files for convex decomposition obj files
  add_custom_command(
    OUTPUT ${basepath}/convex/${basename}.stl
    COMMAND /usr/bin/meshlabserver
    ARGS -i ${it} -o ${basepath}/convex/${basename}.stl
    DEPENDS ${it})

  set(convex_objects_gen_files ${convex_objects_gen_files} ${basepath}/convex/${basename}.stl)
endforeach(it)

add_custom_target(convex_media_files ALL DEPENDS ${convex_objects_gen_files})

