#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: rossync 3844 2009-02-16 19:49:10Z gerkey $
# Modified by Kevin Watts for two arm use

import roslib
import rospy
import rosparam

from robot_msgs.msg import JointTraj, JointTrajPoint

from mechanism_control import mechanism

import sys

def go(side, positions):
  pub = rospy.Publisher(side + '_arm_trajecory_controller/arm_trajectory_command', JointTraj)

  # HACK
  import time
  time.sleep(3.0)

  msg = JointTraj()
  msg.points = []
  for i in range(0,len(positions)):
    msg.points.append(JointTrajPoint())
    msg.points[i].positions = positions[i]
    msg.points[i].time = 0.0

  pub.publish(msg)

USAGE = 'tuckarm.py <arms> ; <arms> is \'(r)ight\', \'(l)eft\', or \'(b)oth\' arms'

spawn_controller = rospy.ServiceProxy('spawn_controller', SpawnController)

def set_params_right():
  rosparam.set_param("right_arm/trajectory_controller/velocity_scaling_factor", 1.0)
  rosparam.set_param("right_arm/trajectory_controller/trajectory_wait_timeout", 0.25)

  rosparam.set_param("right_arm/trajectory_controller/r_shoulder_pan_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("right_arm/trajectory_controller/r_shoulder_lift_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("right_arm/trajectory_controller/r_shoulder_roll_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("right_arm/trajectory_controller/r_elbow_flex_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("right_arm/trajectory_controller/r_forearm_roll_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("right_arm/trajectory_controller/r_wrist_flex_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("right_arm/trajectory_controller/r_wrist_roll_joint/goal_reached_threshold", 0.1)

def set_params_left():
  rosparam.set_param("left_arm/trajectory_controller/velocity_scaling_factor", 1.0)
  rosparam.set_param("left_arm/trajectory_controller/trajectory_wait_timeout", 0.25)

  rosparam.set_param("left_arm/trajectory_controller/l_shoulder_pan_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("left_arm/trajectory_controller/l_shoulder_lift_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("left_arm/trajectory_controller/l_shoulder_roll_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("left_arm/trajectory_controller/l_elbow_flex_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("left_arm/trajectory_controller/l_forearm_roll_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("left_arm/trajectory_controller/l_wrist_flex_joint/goal_reached_threshold", 0.1)
  rosparam.set_param("left_arm/trajectory_controller/l_wrist_roll_joint/goal_reached_threshold", 0.1)


if __name__ == '__main__':
  if len(sys.argv) < 2):
    print USAGE
    sys.exit(-1)

  side = sys.argv[1]

  rospy.init_node('tuck_in', anonymous = True)

  # Positions order
  # s. pan, s. lift, up roll, elbow flex, elbow roll, wrist flex, wrist roll

  path = roslib.packages.get_pkg_dir('sbpl_arm_executive')

  xml_for_left = open(path + '/launch/xml/l_arm_trajectory_controller.xml')
  xml_for_right = open(path + '/launch/xml/r_arm_trajectory_controller.xml')

  controllers = []
  try:
    if side == 'l' or side == 'left':
      # tuck traj for left arm
      set_params_left()
      resp = spawn_controller(xml_for_left.read())
      if ord(resp.ok[0]) != 0:
        controllers.append(resp.name[0])

      positions = [[0.0,0.0,0.0,-2.25,0.0,0.0,0.0], [0.0,1.57,1.57,-2.25,0.0,0.0,0.0]]  
      go('left', positions)

    elif side == 'r' or side == 'right':
      # tuck traj for right arm
      set_params_right()
      resp = spawn_controller(xml_for_right.read())
      if ord(resp.ok[0]) != 0:
        controllers.append(resp.name[0])

      positions = [[0.0,0.0,0.0,-2.25,0.0,0.0,0.0], [0.0,1.57,-1.57,-2.25,0.0,0.0,0.0]]    
      go('right', positions)

    elif side == 'b' or side == 'both':
      # Both arms
      # Holds left arm up at shoulder lift
      set_params_left()
      resp = spawn_controller(xml_for_left.read())
      if ord(resp.ok[0]) != 0:
        controllers.append(resp.name[0])
      set_params_right()
      resp = spawn_controller(xml_for_right.read())
      if ord(resp.ok[0]) != 0:
        controllers.append(resp.name[0]) 
        
      positions_l = [[0.0,0.0,0.0,-2.25,0.0,0.0,0.0], [0.0,1.1,1.57,-2.25,0.0,0.0,0.0]] 
      positions_r = [[0.0,0.0,0.0,-2.25,0.0,0.0,0.0], [0.0,1.57,-1.57,-2.25,0.0,0.0,0.0]]
      
      go('right', positions_r)
      time.sleep(1.0)
      go('left', positions_l)
    else:
      print 'Unknown side! Must be l/left, r/right, or b/both!'
      print USAGE
      sys.exit(2)
  finally:
    for name in controllers:
      mechanism.kill_controller(name)
