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
from people.msg import PositionMeasurement
from people.srv import StartDetection

import visualization_msgs.msg
import tf
import pr2_robot_actions.msg
from tf.listener import TransformListener
import robot_msgs
import python_actions

class FindHelperAction(python_actions.Action, TransformListener):

  def __init__(self, *args):
    TransformListener.__init__(self)
    python_actions.Action.__init__(self, args[0], args[1], args[2], args[3])
    self.name = args[0]
    try:
      self.head_controller = rospy.get_param(self.name + "/head_controller")
    except KeyError:
      self.head_controller = "head_controller"
      rospy.set_param(self.name + "/head_controller", self.head_controller)

    self.head_controller_publisher = rospy.Publisher(self.head_controller + "/set_command_array", robot_msgs.msg.JointCmd)
    self.people_sub = rospy.Subscriber("/face_detection/people_tracker_measurements", PositionMeasurement, self.people_position_measurement)
    self.face_det = rospy.ServiceProxy('/start_detection', StartDetection)

  def people_position_measurement(self, msg):
    self.found = msg

  def execute(self, goal):
    rospy.logdebug("%s: executing.", self.name)
    search_pattern = [ math.pi / 4, 0, -math.pi / 4 ]

    self.found = None
    for angle in search_pattern:
      for timer in range(60): # in tenths
        time.sleep(0.1)
        jc = robot_msgs.msg.JointCmd()
        jc.names = [ "head_pan_joint", "head_tilt_joint" ]
        jc.efforts = [ 0.0, 0.0 ]
        jc.velocity = [ 0.0, 0.0 ]
        jc.acc = [ 0.0, 0.0 ]
        jc.positions = [ angle, 0.0 ]
        self.head_controller_publisher.publish(jc)

        if timer == 10:
          rospy.logdebug("%s: detecting face.", self.name)
          # Run the face detector
          self.face_det()
          rospy.logdebug("%s: returned from service call.", self.name)
        if self.isPreemptRequested():
          rospy.logdebug("%s: preempted.", self.name)
          return python_actions.PREEMPTED
        self.update()
        if self.found:
          ps0 = robot_msgs.msg.PoseStamped()
          ps0.header = self.found.header
          ps0.pose.position.x = self.found.pos.x
          ps0.pose.position.y = self.found.pos.y
          ps0.pose.position.z = self.found.pos.z

          ps0.pose.orientation.x = 0.5
          ps0.pose.orientation.y = -0.5
          ps0.pose.orientation.z = 0.5
          ps0.pose.orientation.w = 0.5

          ps1 = robot_msgs.msg.PoseStamped()
          ps1.header = self.found.header
          ps1.pose.position.x = self.found.pos.x
          ps1.pose.position.y = self.found.pos.y
          ps1.pose.position.z = self.found.pos.z

          ps1.pose.position.z -= 1.25
          ps1.pose.orientation.x = 0.5
          ps1.pose.orientation.y = -0.5
          ps1.pose.orientation.z = 0.5
          ps1.pose.orientation.w = 0.5

          markerpub = rospy.Publisher("/visualization_marker", visualization_msgs.msg.Marker)
          m = visualization_msgs.msg.Marker()
          for i in range(10):
            m.header = ps0.header
            m.ns = "find_helper";
            m.id = i;
            m.type = visualization_msgs.msg.Marker.SPHERE;
            m.action = visualization_msgs.msg.Marker.ADD;
            m.pose = ps0.pose
            m.scale.x = 0.2;
            m.scale.y = 0.2;
            m.scale.z = 0.2;
            m.color.a = 0.5;
            m.color.r = 1.0;
            m.color.g = 0.0;
            m.color.b = 1.0;
            markerpub.publish(m)

          ps0 = self.transformPose("odom_combined", ps0)
          ps1 = self.transformPose("odom_combined", ps1)
          ps1.pose.position.z = 0                             # make sure that the goal is on the ground
          self.feedback.helper_head=ps0
          self.feedback.helper_zone=ps1
        
          for i in range(10):
            m.header = ps1.header
            m.ns = "find_helper";
            m.id = 9+i;
            m.type = visualization_msgs.msg.Marker.SPHERE;
            m.action = visualization_msgs.msg.Marker.ADD;
            m.pose = ps1.pose
            m.scale.x = 0.1;
            m.scale.y = 0.1;
            m.scale.z = 0.1;
            m.color.a = 0.5;
            m.color.r = 0.0;
            m.color.g = 1.0;
            m.color.b = 1.0;
            markerpub.publish(m)

          rospy.logdebug("%s: succeeded.", self.name)
          return python_actions.SUCCESS

    rospy.logdebug("%s: aborted.", self.name)
    return python_actions.ABORTED

#sys.exit()

if __name__ == '__main__':

  try:

    rospy.init_node("find_helper",  log_level=roslib.msg.Log.DEBUG)
    w = FindHelperAction("find_helper", Empty, pr2_robot_actions.msg.FindHelperState, pr2_robot_actions.msg.FindHelperResult)
    w.run()
    rospy.spin();

  except KeyboardInterrupt, e:

    pass

  print "exiting"
