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
from robot_msgs.msg import Planner2DGoal, Planner2DState, ControllerStatus

class MoveBase:
  def __init__(self):
    self.pub_goal = rospy.Publisher("goal", Planner2DGoal)
    rospy.Subscriber("state", Planner2DState, self.stateCallback)
    self.status = None

  def stateCallback(self, msg):
    self.status = msg.status

  def moveBase(self, frame, x, y, a):
    g = Planner2DGoal()
    g.goal.x = x
    g.goal.y = y
    g.goal.th = a
    g.enable = 1
    g.header.frame_id = frame
    g.timeout = 0.0
    print '[MoveBase] Sending the robot to (%f,%f,%f)'%(g.goal.x,g.goal.y,g.goal.th)
    self.pub_goal.publish(g)

    # HACK to get around the lack of proper goal_id support
    print '[MoveBase] Waiting for goal to be taken up...'
    rospy.sleep(2.0)

    while self.status == None or self.status.value == ControllerStatus.ACTIVE:
      print '[MoveBase] Waiting for goal achievement...'
      rospy.sleep(1.0)

    return self.status.value == ControllerStatus.SUCCESS

if __name__ == '__main__':
  import sys
  if len(sys.argv) != 5:
    print 'too few args'
    sys.exit(-1)

  frame = sys.argv[1]
  x = float(sys.argv[2])
  y = float(sys.argv[3])
  a = float(sys.argv[4])
  mb = MoveBase()

  rospy.init_node('move_base', anonymous=True)

  res = mb.moveBase(frame, x, y, a)

  if res:
    print 'Success!'
  else:
    print 'Failure!'
    
