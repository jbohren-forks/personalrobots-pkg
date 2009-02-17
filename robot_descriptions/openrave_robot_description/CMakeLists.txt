cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rospack(openrave_robot_description)

rospack_add_boost_directories()

rospack_add_executable(urdf2openrave src/urdf2openrave.cpp)

# find needed paths
find_ros_package(pr2_defs)
find_ros_package(openrave_robot_description)
find_ros_package(openrave)
get_target_property(urdf2openrave_exe urdf2openrave LOCATION)

set(robot_files )

# process all urdf files
set(PR2_OUTPUT ${openrave_robot_description_PACKAGE_PATH}/robots/pr2.robot.xml)
set(PR2_INPUT ${pr2_defs_PACKAGE_PATH}/robots/pr2.expanded.xml)
set(PR2_GRIPPER_OUTPUT ${openrave_robot_description_PACKAGE_PATH}/robots/pr2_gripper.robot.xml)
set(PR2_GRIPPER_INPUT ${pr2_defs_PACKAGE_PATH}/robots/gripper.expanded.xml)
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

set(PR2_ROBOT "${CMAKE_SOURCE_DIR}/robots/pr2full.robot.xml")
set(PR2_IK_LEFT_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/src/ik_pr2left.h")
add_custom_command(
  OUTPUT ${PR2_IK_LEFT_OUTPUT}
  # need -d 0 to suppress spurious warnings
  COMMAND "${openrave_PACKAGE_PATH}/bin/openrave" -d 0 --generateik robot "${PR2_ROBOT}" freeparam l_shoulder_pan_joint manipulator 0 output "${PR2_IK_LEFT_OUTPUT}"
  DEPENDS ${PR2_ROBOT}
  )
add_custom_target(pr2leftik ALL DEPENDS ${PR2_IK_LEFT_OUTPUT})
add_dependencies(pr2leftik urdf_robots)

set(PR2_IK_RIGHT_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/src/ik_pr2right.h")
add_custom_command(
  OUTPUT ${PR2_IK_RIGHT_OUTPUT}
  # need -d 0 to suppress spurious warnings
  COMMAND "${openrave_PACKAGE_PATH}/bin/openrave" -d 0 --generateik robot "${PR2_ROBOT}" freeparam r_shoulder_pan_joint manipulator 1 output "${PR2_IK_RIGHT_OUTPUT}"
  DEPENDS ${PR2_ROBOT}
  )
add_custom_target(pr2rightik ALL DEPENDS ${PR2_IK_RIGHT_OUTPUT})
add_dependencies(pr2rightik urdf_robots)

# add the plugin for the inverse kinematics
rospack_add_library(orrosik src/orrosikmain.cpp)
add_dependencies(orrosik pr2leftik pr2rightik)
#rospack_link_boost(orrosik thread)

# add the test package
rospack_add_gtest(test_or_robots test/test_robots.cpp)
target_link_libraries (test_or_robots openrave-core)

string(REGEX REPLACE ";" ":" robot_test_files "${robot_files}")
set(robot_test_files "${robot_test_files}:${openrave_robot_description_PACKAGE_PATH}/robots/pr2full.robot.xml:${openrave_robot_description_PACKAGE_PATH}/robots/pr2gripperfull.robot.xml")
set_target_properties(test_or_robots PROPERTIES COMPILE_FLAGS "-DROBOT_FILES=\\\"${robot_test_files}\\\"")
