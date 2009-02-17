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

def callback(data):
  print 'Got state: %d:%d:%d:%d:%d'%(data.active,
                                     data.valid,
                                     data.done,
                                     data.aborted,
                                     data.preempted)

def go():
  pub = rospy.Publisher('right_arm_goal', MoveArmGoal)
  rospy.Subscriber('right_arm_state', MoveArmState, callback)

  rospy.init_node('talker', anonymous=True)
  # HACK
  rospy.sleep(3.0)

  while not rospy.is_shutdown():
    msg = MoveArmGoal()
    msg.configuration = []
    msg.configuration.append(JointState('r_shoulder_pan_joint',-0.5,0.0,0.0,0.0,0))
    msg.enable = 1
    msg.timeout = 0.0

    pub.publish(msg)
    print 'Publishing: ' + `msg.configuration`
    rospy.sleep(3.0)

if __name__ == '__main__':
  go()
