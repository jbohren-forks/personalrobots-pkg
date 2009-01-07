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

import rostools
rostools.update_path('executive_python')
import rospy
from highlevel_controllers.msg import RechargeGoal, RechargeState

class RechargeAdapter:
  def __init__(self, recharge_level, state_topic, goal_topic):
    self.recharge_level = recharge_level
    rospy.Subscriber(state_topic, RechargeState, self.update)
    self.pub = rospy.Publisher(goal_topic, RechargeGoal)
    self.state = None
    self.state_topic = state_topic
    self.goal_topic = goal_topic

  def charge(self, plug_in, pose):
    goal = RechargeGoal()
    goal.recharge_level = self.recharge_level
    goal.enable = 1
    goal.pose.x = pose[0]
    goal.pose.y = pose[1]
    goal.pose.th = pose[2]
    goal.plugIn = plug_in
    self.pub.publish(goal)

  def legalState(self):
    return self.state != None

  def update(self, state):
    self.state = state

  def doneCharging(self):
    return self.state.done == 1

  def charging(self):
    return self.state.active == 1
