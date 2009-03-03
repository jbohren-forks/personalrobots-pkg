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
from robot_srvs.srv import RecordStaticMapTrigger, RecordStaticMapTriggerRequest

from tiltscan import TiltScan

import sys
from math import *

class RecordStaticMap:
  def __init__(self, ts):
    self.ts = ts

  def recordStaticMap(self):
    resp = self.ts.tiltScan(10.0)
    if not resp:
      print '[ApproachTable] Failed to change scan rate'
      return False

    print '[RecordStaticMap] Waiting for service: collision_map_buffer/record_static_map'
    rospy.wait_for_service('collision_map_buffer/record_static_map')
    print '[RecordStaticMap] Calling collision_map_buffer/record_static_map service...'
    s = rospy.ServiceProxy('collision_map_buffer/record_static_map', RecordStaticMapTrigger)
    resp = s.call(RecordStaticMapTriggerRequest(roslib.rostime.Time().from_seconds(resp.time)))
    print '[RecordStaticMap] response: %d' % resp.status
    return resp
  
if __name__ == '__main__':
  ts = TiltScan('laser_tilt_controller', 5.0)
  rsm = RecordStaticMap(ts)

  rospy.init_node('record_static_map', anonymous=True)

  res = rsm.recordStaticMap()

  if not res:
    print 'Failure!'
  else:
    print 'Success!'
    

