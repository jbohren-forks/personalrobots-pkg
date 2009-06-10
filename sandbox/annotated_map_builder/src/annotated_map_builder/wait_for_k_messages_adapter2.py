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

class WaitForKMessagesAdapter:
  def __init__(self, message_topic,msgType, count, timeout):
    print message_topic,msgType
    rospy.Subscriber(message_topic, msgType, self.update)

    self.message_topic = message_topic
    self.time_limit_ = timeout
    self.wait_count_=count;
    self.msg_wait_count_down_=0;

  def update(self,msg):
    print "+",self.msg_wait_count_down_
    if self.msg_wait_count_down_>0:
      self.msg_wait_count_down_ -= 1;

  def doneWaiting(self):
    return self.msg_wait_count_down_==0;

  def startWaiting(self):
    self.start_time_ = rospy.get_time()
    pursuit_time = rospy.get_time() - self.start_time_
    self.msg_wait_count_down_=self.wait_count_

  def timeUp(self):
    pursuit_time = rospy.get_time() - self.start_time_
    return pursuit_time > self.time_limit_;
