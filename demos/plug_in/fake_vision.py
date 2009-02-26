#! /usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('pr2_mechanism_controllers')
roslib.load_manifest('std_msgs')
roslib.load_manifest('rospy')

import rospy
from std_msgs.msg import *
from robot_msgs.msg import *
from tf.msg import tfMessage
from math import *
from time import sleep

def xyz(x, y, z):
  p = Point()
  p.x, p.y, p.z = x, y, z
  return p

def rpy(y, p, r):
  q = Quaternion()
  cosY = cos(y / 2.)
  sinY = sin(y / 2.)
  cosP = cos(p / 2.)
  sinP = sin(p / 2.)
  cosR = cos(r / 2.)
  sinR = sin(r / 2.)
  q.x = cosR*cosP*cosY + sinR*sinP*sinY
  q.y = sinR*cosP*cosY + cosR*sinP*sinY
  q.z = cosR*sinP*cosY + sinR*cosP*sinY
  q.w = cosR*cosP*sinY + sinR*sinP*cosY
  return q


class Tracker:
  def __init__(self, topic, Msg):
    self.sub = rospy.Subscriber(topic, Msg, self.callback)
    self.msg = None

  def callback(self, msg):
    self.msg = msg

mechanism_state = Tracker('/mechanism_state', MechanismState)
def last_time():
  global mechanism_state
  if mechanism_state.msg:
    return mechanism_state.msg.header.stamp
  return 0



pub_outlet = rospy.Publisher('/outlet_pose', PoseStamped)
pub_plug = rospy.Publisher('/plug_pose', PoseStamped)
rospy.init_node('fake_vision', anonymous=True)
while last_time() == 0:
  pass
def send():
  op = PoseStamped()
  op.header.stamp = last_time()
  op.header.frame_id = 'torso_lift_link'
  op.pose.position = xyz(0.7, -0.4, -0.2)
  op.pose.orientation = rpy(0, 0, 0)

  pp = PoseStamped()
  pp.header.stamp = last_time()
  pp.header.frame_id = 'r_gripper_tool_frame'
  pp.pose.position = xyz(0.0, 0.0, 0.0)
  pp.pose.orientation = rpy(0,pi/6,0)

  print "Publishing..."
  for i in range(10):
    pub_outlet.publish(op)
    pub_plug.publish(pp)
    sleep(0.1)

send()
