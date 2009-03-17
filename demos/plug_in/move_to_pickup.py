#!/usr/bin/env python

import roslib
roslib.load_manifest('pr2_mechanism_controllers')
roslib.load_manifest('mechanism_control')
roslib.load_manifest('mechanism_bringup')

import rospy

from robot_msgs.msg import JointTraj, JointTrajPoint
from mechanism_control import mechanism
from robot_mechanism_controllers.srv import *

import sys
from time import sleep

def move(positions):
  pub = rospy.Publisher('right_arm/trajectory_controller/trajectory_command', JointTraj)

  # HACK
  sleep(2)

  msg = JointTraj()
  msg.points = []
  for i in range(0,len(positions)):
    msg.points.append(JointTrajPoint())
    msg.points[i].positions = positions[i]
    msg.points[i].time = 0.0

  pub.publish(msg)
  
def set_params():
  rospy.set_param("right_arm/trajectory_controller/velocity_scaling_factor", 0.75)
  rospy.set_param("right_arm/trajectory_controller/trajectory_wait_timeout", 0.1)

  rospy.set_param("right_arm/trajectory_controller/r_shoulder_pan_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_shoulder_lift_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_shoulder_roll_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_elbow_flex_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_forearm_roll_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_wrist_flex_joint/goal_reached_threshold", 0.1)
  rospy.set_param("right_arm/trajectory_controller/r_wrist_roll_joint/goal_reached_threshold", 0.1)  
  
if __name__ == '__main__':

  rospy.wait_for_service('spawn_controller')
  rospy.init_node('move_to_pickup', anonymous = True)

  # Load xml file for arm trajectory controllers
  path = roslib.packages.get_pkg_dir('sbpl_arm_executive')

  xml_for_right = open(path + '/launch/xml/r_arm_trajectory_controller.xml')

  controllers = []
  try:

    # tuck traj for left arm
    set_params()
    mechanism.spawn_controller(xml_for_right.read())
    controllers.append('right_arm/trajectory_controller')

    positions = [[-0.0118545903883, 1.40015507128, -1.49991771552, -1.94172772826, 0.0131312866653, 0.0962640845608, -0.00644695174672],
                 [-2.04879973213, 1.11987025248, -1.07494474299, -2.05521997047, 1.03679317578, 1.66637122973, 0.151969101557],                
                 [-1.94218984843, 1.11597858279, -0.972790862418, -2.05406188637, 1.13866187982, 1.98224613965, 0.138829401448],
                 [-0.588178005201, 1.02579336793, -1.83716985184, -2.05579901252, 2.47973068494, 1.29075854416, 0.0628192819417]]  
                 
    positions = [positions[i] for i in range(len(positions)) if i % 1 == 0]
    print len(positions)
    move(positions)

    rospy.spin()
    
  finally:
    for name in controllers:
      try:
        mechanism.kill_controller(name)
      except:
        pass
