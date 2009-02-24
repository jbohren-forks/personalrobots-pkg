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
from robot_srvs.srv import FindTable, FindTableRequest
from robot_msgs.msg import Planner2DGoal

import sys

def go():
  pub_goal = rospy.Publisher("goal", Planner2DGoal)
  rospy.init_node('test_node', anonymous=True)

  print 'Waiting for table_object_detector service...'
  rospy.wait_for_service('table_object_detector')
  print 'Calling table_object_detector service...'
  s = rospy.ServiceProxy('table_object_detector', FindTable)
  resp = s.call(FindTableRequest())
  print 'Result:'
  print 'Table (frame %s): %f %f %f %f' % (resp.table.header.frame_id,
                                           resp.table.min_x,
                                           resp.table.max_x,
                                           resp.table.min_y,
                                           resp.table.max_y)
  print '%d objects detected on table' % len(resp.table.objects)
  for o in resp.table.objects:
    print '  (%f %f %f): %f %f %f' % \
       (o.center.x, o.center.y, o.center.z,
        o.max_bound.x - o.min_bound.x,
        o.max_bound.y - o.min_bound.y,
        o.max_bound.z - o.min_bound.z)


  g = Planner2DGoal()
  g.goal.x = resp.table.min_x - 1.5
  g.goal.y = resp.table.min_y - 1.5
  g.goal.th = 0.0
  g.enable = 1
  g.header.frame_id = resp.table.header.frame_id
  g.timeout = 0.0

  print 'Sending the robot to (%f,%f,%f)'%(g.goal.x,g.goal.y,g.goal.th)
  pub_goal.publish(g)

if __name__ == '__main__':
  go()

