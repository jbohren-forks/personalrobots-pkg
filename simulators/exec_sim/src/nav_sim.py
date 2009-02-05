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
#* 
#* Author: Eitan Marder-Eppstein
#***********************************************************
import roslib
roslib.load_manifest('exec_sim')
import rospy
import threading

from robot_msgs.msg import Planner2DGoal, Planner2DState

class NavigationSimulator:
  def __init__(self):
    #create the ros interfaces from the simulator to the world
    rospy.Subscriber(goal_topic, PLanner2DGoal, self.updateGoal)
    self.pub = rospy.Publisher(state_topic, Planner2DState)

    #set the initial state
    self.state = Planner2DState
    self.state.active = 0
    self.state.valid = 0
    self.state.done = 0
    self.state.preempted = 0

    self.state.pos.x = 0.0
    self.state.pos.y = 0.0
    self.state.pos.th = 0.0

    self.state.goal.x = 0.0
    self.state.goal.y = 0.0
    self.state.goal.th = 0.0
    self.state_lock = threading.Lock()

  def setPosition(self, x, y, th):
    self.state.pos.x = x
    self.state.pos.y = y
    self.state.pos.th = th

  def setGoal(self, x, y, th):
    self.state.goal.x = x
    self.state.goal.y = y
    self.state.goal.th = th

  def sendStateMsg(self):
    self.state.header.stamp = rospy.get_rostime()
    self.pub.publish(self.state)

  def updateGoal(self, goal_msg):
    if goal_msg.enable == 1:
      self.state_lock.acquire()
      self.setGoal(goal_msg.goal.x, goal_msg.goal.y, goal_msg.goal.th)
      self.state_lock.release()

  def simulation_step(self):
    self.state_lock.acquire()

    self.state_lock.release()

