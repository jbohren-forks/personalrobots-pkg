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
		  COMMAND ${urdf2gazebo_exe} ${wg_robot_description_PACKAGE_PATH}/pr2/pr2.xml ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml.model
		  DEPENDS urdf2gazebo
		  COMMENT "Building Gazebo model for PR2")

add_custom_target(pr2_gazebo_model_nolimit ALL
		  COMMAND ${urdf2gazebo_exe} ${wg_robot_description_PACKAGE_PATH}/pr2/pr2.xml ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml_nolimit.model 1
		  DEPENDS urdf2gazebo
		  COMMENT "Building Gazebo model for PR2")
add_custom_target(pr2_gazebo_model_test ALL
		  COMMAND ${urdf2gazebo_exe} ${wg_robot_description_PACKAGE_PATH}/pr2/pr2_test.xml ${gazebo_robot_description_PACKAGE_PATH}/world/pr2_xml_test.model 1
		  DEPENDS urdf2gazebo
		  COMMENT "Building Gazebo model for PR2")

