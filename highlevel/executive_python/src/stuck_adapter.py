#! /usr/bin/python

#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2008, Willow Garage, Inc.
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
roslib.load_manifest('executive_python')
import rospy
import random
from pr2_msgs.msg import BaseControllerState
from robot_msgs.msg import PoseDot

class StuckAdapter:
  def __init__(self, state_topic, vel_topic, timeout):
    rospy.Subscriber(state_topic, BaseControllerState, self.update)
    self.pub = rospy.Publisher(vel_topic, PoseDot)
    self.state = None
    self.state_topic = state_topic
    self.goal_topic = vel_topic
    self.timeout = timeout

  def legalState(self):
    return self.state != None

  def update(self, state):
    self.state = state

  def stuck(self):
    for stall_value in self.state.joint_stall:
      if stall_value > 0:
        return True
    return False

  def getUnstuck(self):
    begin = rospy.get_time()
    #to get unstuck... we're going to try to go at max speed forward until we are unstuck
    while self.stuck() and (begin + self.timeout) < rospy.get_time():
      start = rospy.get_time()
      vel = PoseDot()
      vel.vel.vx = 1.0
      vel.vel.vy = 0.0
      vel.ang_vel.vz = 0.0
      #publish the command to the base... hope we don't hit anything
      self.pub.publish(vel)
      end = rospy.get_time()
      sleep_time = 0.01 - (end - start)
      if sleep_time > 0:
        rospy.sleep(sleep_time)

    #make sure to publish zeros when we're done
    vel = PoseDot()
    vel.vel.vx = 0.0
    vel.vel.vy = 0.0
    vel.ang_vel.vz = 0.0
    #publish the command to the base... hope we don't hit anything
    self.pub.publish(vel)
