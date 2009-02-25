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

# Author: Brian Gerkey

import roslib
roslib.load_manifest('executive_python')
import rospy
import random
from pr2_msgs.msg import MoveArmState, MoveArmGoal

class MoveArmAdapter:
  def __init__(self, no_plan_limit, time_limit, state_topic, goal_topic):
    self.no_plan_limit = no_plan_limit
    self.time_limit = time_limit
    rospy.Subscriber(state_topic, MoveArmState, self.update)
    self.pub = rospy.Publisher(goal_topic, MoveArmGoal)
    self.state = None
    self.start_time = rospy.get_time()
    self.last_plan_time = rospy.get_time()
    self.state_topic = state_topic
    self.goal_topic = goal_topic

  def legalState(self):
    return self.state != None

  def update(self, state):
    self.state = state

    #if we have a valid plan, reset the timeout on planning
    if self.active():
      self.last_plan_time = rospy.get_time()

  def active(self):
    return self.state.status == MoveArmState.ACTIVE

  #Have we reached a goal
  def goalReached(self):
    return self.state.status == MoveArmState.INACTIVE

  #Send a new goal to the move arm node
  def sendGoal(self, frame, implicit, state, constraints, enable, timeout):
    self.start_time = rospy.get_time()
    goal = MoveArmGoal()
    goal.header.frame_id = frame
    goal.implicit_goal = implicit
    goal.goal_state = state
    goal.goal_constraints = constraints
    goal.enable = 1
    goal.timeout = timeout

    self.pub.publish(goal)

  #Have we spent too long pursuing a goal or attempting to plan
  def timeUp(self):
    pursuit_time = rospy.get_time() - self.start_time
    no_plan_time = rospy.get_time() - self.last_plan_time
    return pursuit_time > self.time_limit or no_plan_time > self.no_plan_limit 

