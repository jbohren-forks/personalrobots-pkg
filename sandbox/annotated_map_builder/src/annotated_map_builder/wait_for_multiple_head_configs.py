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
from mechanism_msgs.msg import JointStates, JointState


class WaitForMultipleHeadConfigsAdapter:
  def __init__(self, head_configs, single_config_waiter,timeout=10):
    self.configs_=head_configs;

    self.time_limit_ = timeout
    self.wait_count_down_=0;

    self.waiter_=single_config_waiter;

    self.goal_topic_ = "/head_controller/command"

    self.pub = rospy.Publisher(self.goal_topic_, JointCmd)

  def doneWaiting(self):
    return self.wait_count_down_==0;

  def startWaiting(self):
    self.wait_count_down_=len(self.configs_);
    self.current_config_ = 0;
    self.sendHeadConfig();
    self.waiter_.startWaiting();

    self.start_time_ = rospy.get_time()


  def sendHeadConfig(self):
    
      ps = JointState()
      ps.name = 'head_pan_joint'
      ps.position = self.configs_[self.current_config_][0]
      ts = JointState()
      ts.name ='head_tilt_joint'
      ts.position = self.configs_[self.current_config_][1]
      js = JointStates()
      js.joints = [ps, ts]

      self.pub.publish(js);


  def sendDefaultHeadConfig(self):
    
      ps = JointState()
      ps.name = 'head_pan_joint'
      ps.position = 0.0
      ts = JointState()
      ts.name ='head_tilt_joint'
      ts.position = 0.0
      js = JointStates()
      js.joints = [ps, ts]

      self.pub.publish(js);



  def timeUp(self):
    pursuit_time = rospy.get_time() - self.start_time_
    return pursuit_time > self.time_limit_;

  def cycle(self):
    if self.waiter_.doneWaiting():
      self.wait_count_down_ -= 1
      if self.wait_count_down_>0:
        self.current_config_ += 1
        self.sendHeadConfig();
        self.waiter_.startWaiting();

