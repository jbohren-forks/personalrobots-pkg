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
from mechanism_msgs.msg import JointStates, JointState


class MoveHeadAction:
  def __init__(self,node_name):

    self.nn=node_name;#HACK

    self.goals_sub_ = rospy.Subscriber(rospy.resolve_name(node_name)+"/activate", MoveHeadGoal, self.onGoal)
    self.preempt_sub_ = rospy.Subscriber(rospy.resolve_name(node_name)+"/preempt",Empty, self.onPreempt)

    self.state_pub_ = rospy.Publisher(rospy.resolve_name(node_name)+"/feedback", MoveHeadState)

    self.head_goal_topic_ = "/head_controller/command"
    self.head_pub_ = rospy.Publisher(self.head_goal_topic_, JointCmd)

    self.status=ActionStatus();
    self.status.value=ActionStatus.RESET

    try:
      self.time_limit_=rospy.get_param("time_limit")
    except:
      self.time_limit_=1e9;

    self.current_goal_=MoveHeadGoal();
    self.postStatus();


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


  def done(self):
    print "DONE"
    self.status.value=robot_actions.msg.ActionStatus.SUCCESS;
    self.postStatus();


  def doneWaiting(self):
    return self.msg_wait_count_down_==0;


  def onPreempt(self,msg):
    print "preempt",self.nn,self.current_goal_
    self.status.value=robot_actions.msg.ActionStatus.PREEMPTED;
    self.state="idle"


  def abort(self):
    self.state="idle"
    self.status.value=robot_actions.msg.ActionStatus.ABORTED;
    self.postStatus();


  def onGoal(self,msg):
    print "g",self.nn
    self.current_goal_=msg;
    self.start_time_ = rospy.get_time()
    self.pursuit_time = rospy.get_time() - self.start_time_
    self.status.value=ActionStatus.ACTIVE;
    self.state="active"
    self.current_config_ = -1;


    while self.status.value==ActionStatus.ACTIVE and not rospy.is_shutdown():
      self.doCycle();
      rospy.sleep(0.1)
    
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
    print self.state;
    if self.timeUp():
      self.abort();

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
        


  def spin(self):
    while not rospy.is_shutdown():
      self.postStatus()
      rospy.sleep(0.1)

  def postStatus(self):
    success_msg=MoveHeadState();
    success_msg.status=self.status;
    success_msg.goal=self.current_goal_;
    success_msg.header.stamp=rospy.get_rostime();
    self.state_pub_.publish(success_msg);
    
  def timeUp(self):
    self.pursuit_time = rospy.get_time() - self.start_time_
    return self.pursuit_time > self.time_limit_;


  def sendHeadConfig(self):
    if self.status.value!=robot_actions.msg.ActionStatus.ACTIVE:
      return
    ps = JointState()
    ps.name = 'head_pan_joint'
    ps.position = self.head_configs_[self.current_config_][0]
    ts = JointState()
    ts.name ='head_tilt_joint'
    ts.position = self.head_configs_[self.current_config_][1]
    js = JointStates()
    js.joints = [ps, ts]

    self.head_pub_.publish(js);




if __name__ == '__main__':
  try:

    rospy.init_node("move_head_action")
    w=MoveHeadAction("move_head_action");
    w_thread=threading.Thread(None,w.spin)
    w_thread.start()
    
    rospy.spin();

  except KeyboardInterrupt, e:
    pass
  print "exiting"
