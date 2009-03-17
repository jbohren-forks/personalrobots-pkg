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
  rospy.set_param("right_arm/trajectory_controller/trajectory_wait_timeout", 0.25)

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

    positions = [[-0.0169115133318, 1.4054003652, -1.49847441109, -2.05145619713, -0.0319894340348, 0.0976998796058, -0.00781787891648],
                 [-1.74239994197, 1.40641558338, -1.31998576262, -2.05290380226, 1.48152151007, 1.74921225294, 0.199893804267],
                 [-1.7423170416, 1.40345453036, -1.31854245819, -2.05391712585, 1.48152151007, 1.77527410845, 0.173831948753],
                 [-1.3627991197, 1.4054003652, -2.05174111119, -1.98081306678, 2.57315371562, 1.36067741196, -0.749819354622],
                 [-1.30145284137, 1.30311713371, -2.25348299777, -1.9098804154, 2.74935591466, 1.13752005481, -0.527619194165] ]  
                 
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
