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
import robot_msgs.msg
import python_actions




class WriteTrajectoryAction(python_actions.Action):

  def __init__(self, *args):

    python_actions.Action.__init__(self, args[0], args[1], args[2], args[3])
    self.name = args[0]
    
    try:
      self.arm_controller = rospy.get_param(self.name + "/arm_controller")
    except KeyError:
      self.arm_controller = "r_arm_cartesian_pose_controller"
      rospy.set_param(self.name + "/arm_controller", self.arm_controller)

    self.arm_controller_publisher = rospy.Publisher("r_arm_cartesian_pose_controller/command",  robot_msgs.msg.PoseStamped)
    
    
  def execute(self, goal):

    rospy.logdebug("%s: executing.", self.name)

    p = PoseStamped()
    p.header.frame_id = 'white_board_frame'
    p.pose.orientation.x = -0.5
    p.pose.orientation.y = -0.5
    p.pose.orientation.z = 0.5
    p.pose.orientation.w = -0.5 

    for g in goal:
      p.header.stamp = rospy.get_rostime()
      p.pose = g.pose
      self.arm_controller_publisher(p)
      if self.isPreemptRequested():
        rospy.logdebug("%s: preempted.", self.name)
        return python_actions.PREEMPTED
      time.sleep(0.2)
    
   
    rospy.logdebug("%s: succeeded.", self.name)
    return python_actions.SUCCESS


if __name__ == '__main__':

  try:

    rospy.init_node("write_trajectory",  log_level=roslib.msg.Log.DEBUG)
    w = WriteTrajectoryAction("write_trajectory", robot_msgs.msg.Path, robot_actions.msg.WriteTracjectoryState, Empty)
    w.run()
    rospy.spin();
    

  except KeyboardInterrupt, e:

    pass

  print "exiting"
