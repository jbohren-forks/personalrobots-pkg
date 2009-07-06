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
roslib.load_manifest('annotated_map_builder')
import rospy
import random
from pr2_msgs.msg import BaseControllerState
from robot_msgs.msg import PoseDot
from robot_msgs.msg import PoseDot

from annotated_map_builder.msg import *

class WaitForKMessagesAdapter:
  def __init__(self, action_name, count,timeout=10.0):

    self.action_name_=action_name;
    self.state_topic_=self.action_name_+"/feedback";
    print self.state_topic_

    rospy.Subscriber(self.state_topic_, WaitActionState, self.update)
    self.pub_ = rospy.Publisher(self.action_name_+"/request", WaitActionGoal)

    self.wait_count_=count;

    self.time_limit_ = timeout
    self.state=None



  def startWaiting(self):
    self.start_time_ = rospy.get_time()
    pursuit_time = rospy.get_time() - self.start_time_
    self.msg_wait_count_down_=self.wait_count_

  def timeUp(self):
    pursuit_time = rospy.get_time() - self.start_time_
    return pursuit_time > self.time_limit_;

  def sendGoal(self, count=None):
    self.start_time_ = rospy.get_time()
    goal = WaitActionGoal()
    if count:
      goal.num_events = count
    else:
      goal.num_events = self.wait_count_;
    goal.topic_name =  "deprecated";

    self.pub_.publish(goal)


  def legalState(self):
    return self.state != None

  def update(self, state):
    self.state = state
    #print self.state

    #if we have a valid plan, reset the timeout on planning
    if self.state.status.value == self.state.status.ACTIVE:
      self.last_plan_time = rospy.get_time()

  def active(self):
    return self.state.status.value == self.state.status.ACTIVE

  def aborted(self):
    return self.state.status.value == self.state.status.ABORTED

  def success(self):
    return self.state.status.value == self.state.status.SUCCESS

  #Have we reached a goal
  def goalReached(self):
    return self.state.status.value == self.state.status.SUCCESS

