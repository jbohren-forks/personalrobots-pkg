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
import sys
from robot_msgs.msg import PoseStamped
from annotated_map_builder.msg import WaitActionState
from sensor_msgs.msg import RawStereo

from robot_actions.msg import ActionStatus;


class SelectInterestingFrames:
  def __init__(self):

    state_topic="/wait_k_messages_action/feedback"
    data_topic="/stereo/raw_stereo_throttled"

    self.sub_state_ = rospy.Subscriber(state_topic, WaitActionState, self.onState)
    self.sub_image_ = rospy.Subscriber(data_topic, RawStereo, self.onData)

    self.prev_state_ = None
    self.state_="none"
    self.image_list_=[];
    self.all_selections = [];

    self.out_fn = rospy.get_param("~out_file");
    self.fOut=open(self.out_fn,'w')

  def onState(self,state):
    #print "s"
    if state.status.value==ActionStatus.ACTIVE and (not self.prev_state_ or self.prev_state_.status.value != ActionStatus.ACTIVE):
      #Activate
      self.state_="active"
      self.image_list_=[];
    elif state.status.value==ActionStatus.SUCCESS and (self.prev_state_ and self.prev_state_.status.value == ActionStatus.ACTIVE):
      self.state_="idle"
      if len(self.image_list_)>0:
        selection=self.image_list_[len(self.image_list_)/2];
        self.all_selections.append(selection);
        print selection.secs,selection.nsecs
        print >>self.fOut,selection.secs,selection.nsecs

    else:
      self.state="idle"

    self.prev_state_=state

  def onData(self,image_msg):
    #print "d"
    if self.state_=="active":
      self.image_list_.append(image_msg.header.stamp);

if __name__ == '__main__':

  rospy.init_node("select_frames", anonymous=True)
  rg=SelectInterestingFrames()
  rospy.spin();
    
