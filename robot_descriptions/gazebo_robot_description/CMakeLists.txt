cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(gazebo_robot_description)
rospack_add_executable(urdf2gazebo src/urdf2gazebo.cpp)

# find needed paths
find_ros_package(wg_robot_description)
find_ros_package(gazebo_robot_description)
get_target_property(urdf2gazebo_exe urdf2gazebo LOCATION)

# build the ogre mesh files from *.stl (and *.stlb from convex decomposition)
file(GLOB pr2_stl_files ${wg_robot_description_PACKAGE_PATH}/models/pr2/*.stl ${wg_robot_description_PACKAGE_PATH}/models/pr2/convex/*.stlb)
set(pr2_gen_files "")

set(pr2_out_path ${CMAKE_CURRENT_SOURCE_DIR}/world/Media/models/pr2)

MAKE_DIRECTORY(${pr2_out_path})

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

# deprecated by urdf2factory
# add_dependencies(pr2_arm_urdf urdf2gazebo)
