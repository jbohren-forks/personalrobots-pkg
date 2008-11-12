cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(gazebo_robot_description)
rospack_add_executable(urdf2gazebo src/urdf2gazebo.cpp)

# find needed paths
find_ros_package(wg_robot_description)
find_ros_package(gazebo_robot_description)
get_target_property(urdf2gazebo_exe urdf2gazebo LOCATION)

# build the ogre mesh files
file(GLOB pr2_stl_files ${wg_robot_description_PACKAGE_PATH}/models/pr2/*.stl)
set(pr2_gen_files "")

set(pr2_out_path ${CMAKE_CURRENT_SOURCE_DIR}/world/Media/models/pr2)

foreach(it ${pr2_stl_files})
  get_filename_component(basename ${it} NAME_WE)

  # convert to ogre files
  add_custom_command(
    OUTPUT ${pr2_out_path}/${basename}.mesh
    COMMAND rosrun 
    ARGS ogre_tools stl_to_mesh ${it} ${pr2_out_path}/${basename}.mesh
    DEPENDS ${it})

  set(pr2_gen_files ${pr2_gen_files} ${pr2_out_path}/${basename}.mesh)
endforeach(it)

add_custom_target(media_files ALL DEPENDS ${pr2_gen_files})

# process all urdf files
set(PR2_OUTPUT ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml.model)
set(PR2_OUTPUT_NOLIMIT ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml_nolimit.model)
set(PR2_INPUT ${wg_robot_description_PACKAGE_PATH}/pr2/pr2.xml)
add_custom_command(
  OUTPUT ${PR2_OUTPUT}
  COMMAND ${urdf2gazebo_exe}
  ARGS ${PR2_INPUT} ${PR2_OUTPUT}
  DEPENDS ${PR2_INPUT})
add_custom_command(
  OUTPUT ${PR2_OUTPUT_NOLIMIT}
  COMMAND ${urdf2gazebo_exe}
  ARGS ${PR2_INPUT} ${PR2_OUTPUT_NOLIMIT} 1
  DEPENDS ${PR2_INPUT})

add_custom_target(pr2_urdf ALL DEPENDS ${PR2_OUTPUT} ${PR2_OUTPUT_NOLIMIT})
add_dependencies(pr2_urdf urdf2gazebo)
