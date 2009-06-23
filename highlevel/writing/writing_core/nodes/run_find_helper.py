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
from people.msg import PositionMeasurement
from people.srv import StartDetection

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

    self.head_controller_publisher = rospy.Publisher(self.head_controller + "/set_command_array", robot_msgs.msg.JointCmd)
    self.people_sub = rospy.Subscriber("/face_detection/people_tracker_measurements", PositionMeasurement, self.people_position_measurement)
    self.face_det = rospy.ServiceProxy('/start_detection', StartDetection)

  def people_position_measurement(self, msg):
    print "people_position_measurement:"
    print msg
    self.found = msg

  def execute(self, goal):

    search_pattern = [ math.pi / 2, 0, -math.pi / 2 ]

    self.found = None
    for angle in search_pattern:
      for timer in range(60): # in tenths
        time.sleep(0.1)
        print angle, timer
        jc = robot_msgs.msg.JointCmd()
        jc.names = [ "head_pan_joint", "head_tilt_joint" ]
        jc.efforts = [ 0.0, 0.0 ]
        jc.velocity = [ 0.0, 0.0 ]
        jc.acc = [ 0.0, 0.0 ]
        jc.positions = [ angle, 0.0 ]
        self.head_controller_publisher.publish(jc)

        if timer == 10:
          print "detecting face"
          # Run the face detector
          self.face_det(1)
          print "returned from service call"
        if self.isPreemptRequested():
          return python_actions.PREEMPTED
        self.update()
        if self.found:
          self.feedback.header = self.found.header
          self.feedback.pose.position = self.found.pos
          self.feedback.pose.orientation.x = 0.0
          self.feedback.pose.orientation.y = 0.0
          self.feedback.pose.orientation.z = 0.0
          self.feedback.pose.orientation.w = 1.0
          return python_actions.SUCCESS

    return python_actions.ABORTED

#sys.exit()

if __name__ == '__main__':

  try:

    rospy.init_node("find_helper")
    w = FindHelperAction("find_helper", Empty, pr2_robot_actions.msg.FindHelperState, robot_msgs.msg.PoseStamped)
    w.run()
    rospy.spin();

  except KeyboardInterrupt, e:

    pass

  print "exiting"
