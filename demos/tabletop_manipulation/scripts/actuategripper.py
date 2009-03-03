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
from std_msgs.msg import Float64

import sys

class ActuateGripper:
  def __init__(self, side):
    self.side = side
    self.status = None
    self.gripper_close_effort = -0.5
    self.gripper_open_effort = 6.0
    self.pub = rospy.Publisher(self.side + '_gripper/effort_controller/set_command', Float64)
    #rospy.Subscriber(self.side + '_arm_state', MoveArmState, self.movearmCallback)

  def actuateGripper(self, open):
    msg = Float64()
    if open:
      print '[ActuateGripper] Opening gripper...'
      msg.data = self.gripper_open_effort
    else:
      print '[ActuateGripper] Closing gripper...'
      msg.data = self.gripper_close_effort
    self.pub.publish(msg)

    # HACK: don't know what to monitor to determine that the gripper is
    # done opening or closing
    print '[ActuateGripper] Waiting for gripper to finish moving...'
    rospy.sleep(4.0)

    return True

USAGE = 'actuategripper.py {right|left} {open|close}'
if __name__ == '__main__':
  if len(sys.argv) != 3 or \
     (sys.argv[1] != 'right' and sys.argv[1] != 'left') or \
     (sys.argv[2] != 'open' and sys.argv[2] != 'close'):
    print USAGE
    sys.exit(-1)

  side = sys.argv[1]
  open = sys.argv[2] == 'open'

  ag = ActuateGripper(side)

  rospy.init_node('actuate_gripper', anonymous=True)

  # HACK
  import time
  time.sleep(2.0)

  res = ag.actuateGripper(open)

  if res:
    print 'Success!'
  else:
    print 'Failure!'
