#! /usr/bin/python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2009, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

import roslib
import roslib.msg
roslib.load_manifest('writing_core')
import rospy

import time
import sys
import math

from std_msgs.msg import Empty
from robot_actions.msg import NoArgumentsActionState
import robot_actions.msg
import std_msgs.msg
import pr2_robot_actions.msg
import robot_msgs.msg
import python_actions
import mechanism_msgs.msg



class GripPenAction(python_actions.Action):

  def __init__(self, *args):
    python_actions.Action.__init__(self, args[0], args[1], args[2], args[3])
    self.name = args[0]
    
    try:
      self.gripper_controller = rospy.get_param(self.name + "/gripper_controller")
    except KeyError:
      self.gripper_controller = "r_gripper_effort_controller"
      rospy.set_param(self.name + "/gripper_controller", self.gripper_controller)

    self.gripper_controller_publisher = rospy.Publisher(self.gripper_controller + "/command", std_msgs.msg.Float64)
    rospy.Subscriber("joint_states", mechanism_msgs.msg.JointStates, self.check_grip)
    
  def execute(self, goal):

    rospy.logdebug("%s: executing.", self.name)
    self.last_position = 10
    self.grasp_count = 0   
    self.success = -1
    started = time.time()

    while (time.time() - started) < 20.0 and self.success != 0:
      time.sleep(0.1)
      if self.success == 1:
        rospy.logdebug("%s: succeeded.", self.name)
        return python_actions.SUCCESS
    
      if self.isPreemptRequested():
        rospy.logdebug("%s: preempted.", self.name)
        return python_actions.PREEMPTED

    print "aborted due to time out"
    rospy.logdebug("%s: arborted.", self.name)   
    return python_actions.ABORTED


  def check_grip(self, joint_state_msg):

    if self.status != python_actions.ACTIVE:
      return
    
    for j in joint_state_msg.joints:
      joint = j.name
      if joint == "r_gripper_joint":
        if self.last_position > j.position:
          self.last_position = j.position
        else :
          self.grasp_count += 1
        if j.position < 0.035:
          self.gripper_controller_publisher.publish(std_msgs.msg.Float64(-20))

        if self.grasp_count > 500:
          if j.position < 0.015:
            self.success  = 1
          else:
            self.success = 0

            

        
        
        

if __name__ == '__main__':

  try:

    rospy.init_node("grip_pen",  log_level=roslib.msg.Log.DEBUG)
    w = GripPenAction("grip_pen", Empty, robot_actions.msg.NoArgumentsActionState, Empty)
    w.run()
    rospy.spin();

  except KeyboardInterrupt, e:

    pass

  print "exiting"
