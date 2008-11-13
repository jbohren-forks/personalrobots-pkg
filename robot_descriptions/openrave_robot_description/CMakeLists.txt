cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(openrave_robot_description)
rospack_add_executable(urdf2openrave src/urdf2openrave.cpp)

# find needed paths
find_ros_package(wg_robot_description)
find_ros_package(openrave_robot_description)
get_target_property(urdf2openrave_exe urdf2openrave LOCATION)

find_package( Boost COMPONENTS regex )
if( NOT ${Boost_regex_FOUND} )
    message(SEND_ERROR "could not find boost-regex")  
endif()

include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})
target_link_libraries(urdf2openrave ${Boost_LIBRARIES})

# process all urdf files
set(PR2_OUTPUT ${openrave_robot_description_PACKAGE_PATH}/robots/pr2.robot.xml)
set(PR2_INPUT ${wg_robot_description_PACKAGE_PATH}/pr2/pr2.xml)
add_custom_command(
  OUTPUT ${PR2_OUTPUT}
  COMMAND ${urdf2openrave_exe}
  ARGS ${PR2_INPUT} ${PR2_OUTPUT} ../models
  DEPENDS ${PR2_INPUT} urdf2openrave)

add_custom_target(pr2_urdf ALL DEPENDS ${PR2_OUTPUT})
add_dependencies(pr2_urdf urdf2openrave)
