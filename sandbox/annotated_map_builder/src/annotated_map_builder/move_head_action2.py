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
from sensor_msgs.msg import RawStereo
from std_msgs.msg import Empty

import threading

from robot_actions.msg import ActionStatus;

from annotated_map_builder.msg import *
from annotated_map_builder import *
from annotated_map_builder.wait_for_k_messages_adapter import WaitForKMessagesAdapter 

from pr2_msgs.msg import BaseControllerState
from robot_msgs.msg import PoseDot
from robot_msgs.msg import JointCmd

from python_actions import *

class MoveHeadAction2 (Action) :


  def __init__(self,node_name, goalmsg, statemsg, feedbackmsg, rate = 10.0):
    Action.__init__(self, node_name, goalmsg, statemsg, feedbackmsg, rate);

    self.head_goal_topic_ = "/head_controller/set_command_array"
    self.head_pub_ = rospy.Publisher(self.head_goal_topic_, JointCmd)

    try:
      self.time_limit_=rospy.get_param("time_limit")
    except:
      self.time_limit_=1e9;
    try:
      self.wait_action_name_ = rospy.get_param("~wait_action");
    except:
      self.wait_action_name_ = "wait_k_messages_action"

    self.capture_waiter_ = WaitForKMessagesAdapter(self.wait_action_name_,3)

    self.state="idle"

    self.head_configs_=eval(rospy.get_param("~head_configs"));

    print self.head_configs_
    try:
      self.use_random_=rospy.get_param("~use_random");
    except:
      self.use_random_=0;
    print self.head_configs_

    self.current_config_ = 0

    print "INIT done"


  def execute(self, goal):
    count = 1
    self.start_time_ = rospy.get_time()
    self.pursuit_time = rospy.get_time() - self.start_time_
    self.current_config_ = -1;
    self.state="active";

    while not self.isPreemptRequested() and not self.status==ACTIVE:
      self.doCycle();
      time.sleep(0.1)
      print "PYTHON HELLO, count is", count
      count += 1
      self.update()
    if self.isPreemptRequested():
      return python_actions.PREEMPTED
    else:
      return python_actions.SUCCESS


  def timeUp(self):
    self.pursuit_time = rospy.get_time() - self.start_time_
    return self.pursuit_time > self.time_limit_;

  def abort(self):
    self.status=ABORTED;



  def legalStates(self):
    return self.capture_waiter_.legalState()


  def selectNextConfig(self):
    if self.use_random_==1:
      self.current_config_ =random.randint(0, len(self.head_configs_) - 1);
    else:
      self.current_config_ += 1
      if self.current_config_>=len(self.head_configs_):
        self.current_config_=0;

        
  def doCycle(self):
    if self.timeUp():
      self.abort();
      return;

    if not self.legalStates():
      print("Waiting on %s to be published" % (self.capture_waiter_.state_topic_))
      return

    if self.state=="active":
      self.selectNextConfig();
      self.sendHeadConfig();
      self.state="waiting";
      self.capture_waiter_.sendGoal();
      
    elif self.state=="waiting":
      if self.capture_waiter_.timeUp() or self.capture_waiter_.success():
        self.state="active"
      else:
        self.sendHeadConfig();
        

  def sendHeadConfig(self):
    if self.status.value!=robot_actions.msg.ActionStatus.ACTIVE:
      return
    joint_cmds=JointCmd();
    joint_cmds.names=[ "head_pan_joint", "head_tilt_joint"];
    joint_cmds.positions=self.head_configs_[self.current_config_];
    self.head_pub_.publish(joint_cmds);



if __name__ == '__main__':
  try:

    rospy.init_node("move_head_action")
    w=MoveHeadAction2("move_head_action", MoveHeadGoal, MoveHeadState, std_msgs.Empty);
    w.run()
    
    rospy.spin();

  except KeyboardInterrupt, e:
    pass
  print "exiting"
