cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(openrave_robot_description)
rospack_add_executable(urdf2openrave src/urdf2openrave.cpp)

# find needed paths
find_ros_package(wg_robot_description)
find_ros_package(openrave_robot_description)
find_ros_package(openrave)
get_target_property(urdf2openrave_exe urdf2openrave LOCATION)

set(robot_files )

# process all urdf files
set(PR2_OUTPUT ${openrave_robot_description_PACKAGE_PATH}/robots/pr2.robot.xml)
set(PR2_INPUT ${wg_robot_description_PACKAGE_PATH}/pr2/pr2.xml)
set(PR2_GRIPPER_OUTPUT ${openrave_robot_description_PACKAGE_PATH}/robots/pr2_gripper.robot.xml)
set(PR2_GRIPPER_INPUT ${wg_robot_description_PACKAGE_PATH}/pr2_gripper/pr2_gripper.xml)
set(MODELS ${openrave_robot_description_PACKAGE_PATH}/models)

## create the models directory
add_custom_command(
  OUTPUT ${MODELS}
  COMMAND mkdir -p ${MODELS}
)

add_custom_command(
  OUTPUT ${PR2_OUTPUT}
  COMMAND ${urdf2openrave_exe}
  ARGS ${PR2_INPUT} ${PR2_OUTPUT} ../models
  DEPENDS  ${PR2_INPUT} ${MODELS} urdf2openrave)
add_custom_command(
  OUTPUT ${PR2_GRIPPER_OUTPUT}
  COMMAND ${urdf2openrave_exe}
  ARGS ${PR2_GRIPPER_INPUT} ${PR2_GRIPPER_OUTPUT} ../models
  DEPENDS  ${PR2_GRIPPER_INPUT} ${MODELS} urdf2openrave)
set(robot_files ${robot_files} ${PR2_OUTPUT} ${PR2_GRIPPER_OUTPUT})

add_custom_target(urdf_robots ALL DEPENDS ${robot_files})
add_dependencies(urdf_robots urdf2openrave)

# add the test package
rospack_add_gtest(test_or_robots test/test_robots.cpp)
target_link_libraries (test_or_robots openrave-core)

string(REGEX REPLACE ";" ":" robot_test_files "${robot_files}")
set(robot_test_files "${robot_test_files}:${openrave_robot_description_PACKAGE_PATH}/robots/pr2full.robot.xml:${openrave_robot_description_PACKAGE_PATH}/robots/pr2gripperfull.robot.xml")
set_target_properties(test_or_robots PROPERTIES COMPILE_FLAGS "-DROBOT_FILES=\\\"${robot_test_files}\\\"")
