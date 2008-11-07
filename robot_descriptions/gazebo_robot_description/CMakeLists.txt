cmake_minimum_required(VERSION 2.6)
include(rosbuild)
rospack(gazebo_robot_description)
rospack_add_executable(urdf2gazebo src/urdf2gazebo.cpp)

# find needed paths
find_ros_package(wg_robot_description)
find_ros_package(gazebo_robot_description)
get_target_property(urdf2gazebo_exe urdf2gazebo LOCATION)

message(${urdf2gazebo_exe})
message(${wg_robot_description_PACKAGE_PATH}/pr2/pr2.xml)
message(${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml.model)

# we do not know all the dependencies of pr2.xml so we always build the target
add_custom_target(pr2_gazebo_model ALL
		  COMMAND ${urdf2gazebo_exe} ${wg_robot_description_PACKAGE_PATH}/pr2/gazebo/pr2_gazebo_actuators.xml ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml.model
		  DEPENDS urdf2gazebo
		  COMMENT "Building Gazebo model for PR2 Standard Model")

add_custom_target(pr2_gazebo_model_nolimit ALL
		  COMMAND ${urdf2gazebo_exe} ${wg_robot_description_PACKAGE_PATH}/pr2/gazebo/pr2_gazebo_actuators.xml ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml_nolimit.model 1
		  DEPENDS urdf2gazebo
		  COMMENT "Building Gazebo model for PR2 Gazebo Actuators with no limits")

#pr2_arm
add_custom_target(pr2d2 ALL
		  COMMAND ${urdf2gazebo_exe} ${wg_robot_description_PACKAGE_PATH}/pr2_arm/pr2d2.xml ${gazebo_robot_description_PACKAGE_PATH}/world/pr2d2_xml.model
		  DEPENDS urdf2gazebo
		  COMMENT "Building Gazebo model for PR2D2 Arm Test")

# pr2_arm_test
add_custom_target(pr2_gazebo_model_arm ALL
		  COMMAND ${urdf2gazebo_exe} ${wg_robot_description_PACKAGE_PATH}/pr2_arm_test/gazebo/pr2_arm_gazebo_actuators.xml ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml_arm.model
		  DEPENDS urdf2gazebo
		  COMMENT "Building Gazebo model for PR2 Arm Test")

# single_link_test
add_custom_target(pr2_gazebo_model_single_link ALL
		  COMMAND ${urdf2gazebo_exe} ${wg_robot_description_PACKAGE_PATH}/single_link_test/gazebo/pr2_single_link_gazebo_actuators.xml ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml_single_link.model
		  DEPENDS urdf2gazebo
		  COMMENT "Building Gazebo model for Single Link Test")

# dual_link_test
add_custom_target(pr2_gazebo_model_dual_link ALL
		  COMMAND ${urdf2gazebo_exe} ${wg_robot_description_PACKAGE_PATH}/dual_link_test/gazebo/pr2_dual_link_gazebo_actuators.xml ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml_dual_link.model
		  DEPENDS urdf2gazebo
		  COMMENT "Building Gazebo model for Dual Link Test")

# multi_link_test
add_custom_target(pr2_gazebo_model_multi_link ALL
		  COMMAND ${urdf2gazebo_exe} ${wg_robot_description_PACKAGE_PATH}/multi_link_test/gazebo/pr2_multi_link_gazebo_actuators.xml ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml_multi_link.model
		  DEPENDS urdf2gazebo
		  COMMENT "Building Gazebo model for Multi Link Test")

# pr2_prototype1
add_custom_target(pr2_prototype1 ALL
		  COMMAND ${urdf2gazebo_exe} ${wg_robot_description_PACKAGE_PATH}/pr2_prototype1/gazebo/pr2_prototype1_gazebo_actuators.xml ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml_prototype1.model
		  DEPENDS urdf2gazebo
		  COMMENT "Building Gazebo model for PR2 Prototype1 Test")


