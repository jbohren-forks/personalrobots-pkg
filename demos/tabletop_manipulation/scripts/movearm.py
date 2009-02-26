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
from pr2_msgs.msg import MoveArmGoal, MoveArmState
from robot_msgs.msg import JointState

import sys

class MoveArm:
  def __init__(self, side):
    self.side = side
    self.status = None
    self.pub = rospy.Publisher(self.side + '_arm_goal', MoveArmGoal)
    rospy.Subscriber(self.side + '_arm_state', MoveArmState, self.movearmCallback)

  def movearmCallback(self, msg):
    #print '[MoveArm] Got status: %d'%(msg.status)
    self.goal_id = msg.goal_id
    self.status = msg.status

  def moveArm(self, joints):
    msg = MoveArmGoal()
    msg.configuration = []
    for j in joints:
      msg.configuration.append(JointState(j,joints[j],0.0,0.0,0.0,0))
    msg.enable = 1
    msg.timeout = 0.0
    self.pub.publish(msg)
    print '[MoveArm] Sending arm to: ' + `msg.configuration`

    # HACK to get around the lack of proper goal_id support
    print '[MoveArm] Waiting for goal to be taken up...'
    rospy.sleep(2.0)

    while self.status == None or self.status == MoveArmState.ACTIVE:
      print '[MoveArm] Waiting for goal achievement...'
      rospy.sleep(1.0)

    return self.status == MoveArmState.INACTIVE
  
USAGE = 'movearm.py {left|right} <shoulder_lift> <shoulder_pan>'
if __name__ == '__main__':
  if len(sys.argv) != 4 or (sys.argv[1] != 'left' and sys.argv[1] != 'right'):
    print USAGE
    sys.exit(-1)

  side = sys.argv[1]
  joints = {} 
  joints[side[0] + '_shoulder_lift_joint']  = float(sys.argv[2])
  joints[side[0] + '_shoulder_pan_joint']  = float(sys.argv[3])

  ma = MoveArm(side)

  rospy.init_node('talker', anonymous=True)

  # HACK
  import time
  time.sleep(2.0)

  res = ma.moveArm(joints)

  if res:
    print 'Success!'
  else:
    print 'Failure!'
