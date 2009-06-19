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
import threading 

from image_msgs.msg import RawStereo
from std_msgs.msg import Empty


from robot_actions.msg import ActionStatus;

from annotated_map_builder.msg import *

class WaitForKMessagesAction:
  def __init__(self,node_name):

    self.nn=node_name;#HACK

    #try:
    #  self.message_topic_ = rospy.get_param("topic");
    #except:
    #  self.message_topic_ = "/stereo/raw_stereo_throttled"
    self.message_topic_ = "topic" #use remap

    self.goals_sub_ = rospy.Subscriber(self.nn+"/request", WaitActionGoal, self.onGoal)
    print self.goals_sub_
    self.preempt_sub_ = rospy.Subscriber(self.nn+"/preempt",Empty, self.onPreempt)

    self.state_pub_ = rospy.Publisher(self.nn+"/feedback", WaitActionState)

    try:
      self.time_limit_ = rospy.get_param("time_limit");
    except:
      self.time_limit_ = 10;

    self.status=ActionStatus();
    self.status.value=ActionStatus.RESET
    self.postStatus();

    self.topic_sub_ = rospy.Subscriber(self.message_topic_, RawStereo, self.update)

    try:
      self.wait_count_=rospy.get_param("count")
    except:
      self.wait_count_=3;
    
    print "INIT done"


  def update(self,msg):
    print "."
    if self.status.value != ActionStatus.ACTIVE:
      return

    print "+",self.msg_wait_count_down_
    if self.msg_wait_count_down_>0:
      self.msg_wait_count_down_ -= 1;
    else:
      self.done();

  def done(self):
    print "DONE"
    self.status.value=robot_actions.msg.ActionStatus.SUCCESS;
    self.postStatus();


  def doneWaiting(self):
    return self.msg_wait_count_down_==0;


  def onPreempt(self):
    self.status.value=robot_actions.msg.ActionStatus.PREEMPTED;
    if self.topic_sub_:
      self.topic_sub_ = None

    self.postStatus()


  def abort(self):
    self.status.value=robot_actions.msg.ActionStatus.ABORTED;
    if self.topic_sub_:
      self.topic_sub_ = None

    self.postStatus();


  def onGoal(self,msg):
    print msg
    self.wait_count_=msg.num_events
    self.message_topic_=msg.topic_name

    self.msg_wait_count_down_=self.wait_count_

    self.start_time_ = rospy.get_time()
    self.pursuit_time = rospy.get_time() - self.start_time_
    self.status.value=ActionStatus.ACTIVE;

    while self.status.value==ActionStatus.ACTIVE and not rospy.is_shutdown():
      self.doCycle();
      rospy.sleep(0.1)


  def doCycle(self):
    if self.timeUp():
      self.abort();
    self.postStatus();

  def spin(self):
    while not rospy.is_shutdown():
      self.postStatus()
      rospy.sleep(0.1)

  def postStatus(self):
    success_msg=WaitActionState();
    success_msg.status=self.status;
    self.state_pub_.publish(success_msg);
    
  def timeUp(self):
    self.pursuit_time = rospy.get_time() - self.start_time_
    return self.pursuit_time > self.time_limit_;


if __name__ == '__main__':
  try:

    rospy.init_node("wait_k_msg_action")
    w=WaitForKMessagesAction("wait_k_messages_action");
    w_thread=threading.Thread(None,w.spin)
    w_thread.run()
    
    rospy.spin();

  except KeyboardInterrupt, e:
    pass
  print "exiting"
