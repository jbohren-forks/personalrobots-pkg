#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: rossync 3844 2009-02-16 19:49:10Z gerkey $

import roslib
roslib.load_manifest('tabletop_manipulation')
import rospy
from robot_actions.msg import Pose2D, MoveBaseState

class StopBase:
  def __init__(self):
    self.pub_goal = rospy.Publisher("/move_base_node/preempt", Pose2D)
    rospy.Subscriber("/move_base_node/feedback", MoveBaseState, self.stateCallback)
    self.goal_id = -1
    self.status = None

  def stateCallback(self, msg):
    self.goal_id = msg.goal_id
    self.status = msg.status

  def stopBase(self):
    g = Pose2D()
    g.x = 0
    g.y = 0
    g.th = 0
    g.header.frame_id = ''
    print '[StopBase] Stopping base'
    self.pub_goal.publish(g)

    while self.status == None or self.status == self.status.ACTIVE:
      print '[StopBase] Waiting for goal achievement...'
      rospy.sleep(1.0)

    return self.status != self.status.INACTIVE

if __name__ == '__main__':
  import sys
  if len(sys.argv) != 1:
    print 'wrong # args'
    sys.exit(-1)

  sb = StopBase()

  rospy.init_node('stop_base', anonymous=True)

  res = sb.stopBase()

  if res:
    print 'Success!'
  else:
    print 'Failure!'
    
