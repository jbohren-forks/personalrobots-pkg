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
from pr2_mechanism_controllers.srv import SetProfile, SetProfileRequest

class TiltScan:
  def __init__(self, controller, laser_buffer_time):
    self.controller = controller
    self.laser_buffer_time = laser_buffer_time

  def tiltScan(self, period, amplitude, offset):
    svcname = self.controller + '/set_profile'
    print 'Waiting for service ' + svcname
    rospy.wait_for_service(svcname)
    s = rospy.ServiceProxy(svcname, SetProfile)
    resp = s.call(SetProfileRequest(0.0, 0.0, 0.0, 0.0, 
                                    'sine', period, amplitude, offset))
        
    if resp:
      # Set the collision_map_buffer's window size accordingly, to remember a
      # fixed time window scans.
      rospy.client.set_param('collision_map_buffer/window_size', 
                             int(self.laser_buffer_time / (period / 2.0)))

      # Wait until the laser's swept through most of its period at the new
      # speed
      rospy.sleep(.75 * period)

    return resp

if __name__ == '__main__':
  import sys
  if len(sys.argv) != 4:
    print 'too few args'
    sys.exit(-1)

  period = float(sys.argv[1])
  amplitude = float(sys.argv[2])
  offset = float(sys.argv[3])
  ts = TiltScan('laser_tilt_controller', 5.0)

  rospy.init_node('move_base', anonymous=True)

  res = ts.tiltScan(period, amplitude, offset)

  if res:
    print 'Success!'
  else:
    print 'Failure!'
    
