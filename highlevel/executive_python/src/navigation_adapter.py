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
from nav_robot_actions.msg import MoveBaseState
from robot_msgs.msg import PoseStamped

class NavigationAdapter:
  def __init__(self, no_plan_limit, time_limit, state_topic, goal_topic):
    self.no_plan_limit = no_plan_limit
    self.time_limit = time_limit
    rospy.Subscriber(state_topic, MoveBaseState, self.update)
    self.pub = rospy.Publisher(goal_topic, PoseStamped)
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
    if self.state.status.value == self.state.status.ACTIVE:
      self.last_plan_time = rospy.get_time()

  def active(self):
    return self.state.status.value == self.state.status.ACTIVE

  #Have we reached a goal
  def goalReached(self):
    return self.state.status.value == self.state.status.SUCCESS

  def currentPosition(self):
    return [self.state.feedback.x, self.state.feedback.y, self.state.feedback.th]

  def currentGoal(self):
    return [self.state.goal.x, self.state.goal.y, self.state.goal.th]

  #Send a new goal to the move base node
  def sendGoal(self, goal_pts, frame):
    self.start_time = rospy.get_time()
    goal = PoseStamped()
    goal.header.frame_id = frame
    goal.pose.position.x = goal_pts[0][0]
    goal.pose.position.y = goal_pts[0][1]
    goal.pose.position.z = goal_pts[0][2]
    goal.pose.orientation.x = goal_pts[1][0]
    goal.pose.orientation.y = goal_pts[1][1]
    goal.pose.orientation.z = goal_pts[1][2]
    goal.pose.orientation.w = goal_pts[1][3]

    self.pub.publish(goal)

  #Have we spent too long pursuing a goal or attempting to plan
  def timeUp(self):
    pursuit_time = rospy.get_time() - self.start_time
    no_plan_time = rospy.get_time() - self.last_plan_time
    return pursuit_time > self.time_limit or no_plan_time > self.no_plan_limit 

