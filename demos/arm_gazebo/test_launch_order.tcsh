#!/usr/bin/tcsh

echo "----------------starting roscore"
$ROS_ROOT/bin/roscore &

sleep 5

echo "----------------starting gazebo"
source `rospack find gazebo`/setup.tcsh
gazebo `rospack find gazebo_robot_description`/gazebo_worlds/empty.world &

sleep 5


echo "----------------roslaunch xml"
`rospack find xacro`/xacro.py `rospack find pr2_defs`/robots/r_arm.xacro.xml > pr2_arm.xml
#$ROS_ROOT/bin/rosparam set robotdesc/pr2
python ./setparam.py

echo "----------------urdf2factory"
`rospack find gazebo_plugin`/urdf2factory robotdesc/pr2 

echo "----------------spawn controller"
`rospack find mechanism_control`/scripts/mech.py sp `rospack find arm_gazebo`/r_arm_default_controller.xml

echo "----------------set gripper gap"
`rospack find robot_mechanism_controllers`/scripts/control.py set r_gripper_controller 0.0


