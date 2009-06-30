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

class ActuateTorso:
  def __init__(self):
    self.status = None
    self.torso_down_effort = -4000.0
    self.torso_up_effort = 4000.0
    self.pub = rospy.Publisher('torso_lift/effort_controller/set_command', Float64)
    #rospy.Subscriber(self.side + '_arm_state', MoveArmState, self.movearmCallback)

  def actuateTorso(self, up):
    msg = Float64()
    if up:
      print '[ActuateTorso] Raising torso...'
      msg.data = self.torso_up_effort
    else:
      print '[ActuateTorso] Lowering torso...'
      msg.data = self.torso_down_effort
    self.pub.publish(msg)

    # HACK: don't know what to monitor to determine that the torso is
    # done raising or lowering
    print '[ActuateTorso] Waiting for torso to finish moving...'
    rospy.sleep(4.0)

    return True

USAGE = 'actuatetorso.py {up|down}'
if __name__ == '__main__':
  if len(sys.argv) != 2 or \
     (sys.argv[1] != 'up' and sys.argv[1] != 'down'):
    print USAGE
    sys.exit(-1)

  up = sys.argv[1] == 'up'

  ag = ActuateTorso()

  rospy.init_node('actuate_torso', anonymous=True)

  # HACK
  import time
  time.sleep(2.0)

  res = ag.actuateTorso(up)

  if res:
    print 'Success!'
  else:
    print 'Failure!'
