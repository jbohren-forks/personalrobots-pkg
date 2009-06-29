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
roslib.load_manifest('writing_core')
import rospy

import time
import sys
import math

from std_msgs.msg import Empty

from robot_actions.msg import NoArgumentsActionState
import pr2_robot_actions.msg
import robot_msgs
import python_actions

class FindHelperAction(python_actions.Action):

  def __init__(self, *args):
    python_actions.Action.__init__(self, args[0], args[1], args[2], args[3])
    name = args[0]
    try:
      self.head_controller = rospy.get_param(name + "/head_controller")
    except KeyError:
      self.head_controller = "head_controller"
      rospy.set_param(name + "/head_controller", self.head_controller)

    self.head_controller_publisher = rospy.Publisher(self.head_controller + "/head_track_point", robot_msgs.msg.PointStamped)

  def execute(self, goal):

    rospy.logdebug("%s: executing.", self.name)
    htp = robot_msgs.msg.PointStamped()
    htp.header = goal.header
    htp.point = goal.pose.position

    self.head_controller_publisher.publish(htp)

    while(!self.isPreemptRequested()):
      time.sleep(0.1)
      
      htp.header.stamp = rospy.get_rostime()
      self.head_controller_publisher.publish(htp)
      
      self.update()

    rospy.logdebug("%s: preempted.", self.name)    
    return python_actions.PREEMPTED
        
        

#sys.exit()

if __name__ == '__main__':

  try:

    rospy.init_node("track_helper")
    w = TrackHelperAction("track_helper", robot_msgs.msg.PoseStamped, pr2_robot_actions.msg.TrackHelperState, Empty)
    w.run()
    rospy.spin();

  except KeyboardInterrupt, e:

    pass

  print "exiting"
