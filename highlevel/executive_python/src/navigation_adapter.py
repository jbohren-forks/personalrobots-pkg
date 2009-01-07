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
import random
from std_msgs.msg import Planner2DGoal, Planner2DState

class NavigationAdapter:
  def __init__(self, no_plan_limit, time_limit, state_topic, goal_topic):
    self.no_plan_limit = no_plan_limit
    self.time_limit = time_limit
    rospy.Subscriber(state_topic, Planner2DState, self.update)
    self.pub = rospy.Publisher(goal_topic, Planner2DGoal)
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
    if self.state.valid == 1:
      self.last_plan_time = rospy.get_time()

  def active(self):
    return self.state.active == 1

  #Have we reached a goal
  def goalReached(self):
    return self.state.done == 1

  def currentPosition(self):
    return [self.state.pos.x, self.state.pos.y, self.state.pos.th]

  def currentGoal(self):
    return [self.state.goal.x, self.state.goal.y, self.state.goal.th]

  #Send a new goal to the move base node
  def sendGoal(self, goal_pts):
    self.start_time = rospy.get_time()
    goal = Planner2DGoal()
    goal.header.frame_id = "map"
    goal.goal.x = goal_pts[0]
    goal.goal.y = goal_pts[1]
    goal.goal.th = goal_pts[2]
    goal.enable = 1

    self.pub.publish(goal)

  #Have we spent too long pursuing a goal or attempting to plan
  def timeUp(self):
    pursuit_time = rospy.get_time() - self.start_time
    no_plan_time = rospy.get_time() - self.last_plan_time
    return pursuit_time > self.time_limit or no_plan_time > self.no_plan_limit 

