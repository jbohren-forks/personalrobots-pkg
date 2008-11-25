#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

## Gazebo test base controller vw
##   sends cmd_vel vx= 0, vy=0, vw =TARGET_VW
##   checks to see if P3D returns corresponding ground truth within TARGET_TOL of TARGET_VW
##          for a duration of TARGET_DURATION seconds

PKG = 'gazebo_plugin'
NAME = 'testbase_vw_gt'

import math
import rostools
rostools.update_path(PKG)
rostools.update_path('rostest')
rostools.update_path('std_msgs')
rostools.update_path('robot_msgs')
rostools.update_path('rostest')
rostools.update_path('rospy')


import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *


TARGET_VW       =  0.5
TARGET_DURATION = 2.0
TARGET_TOL      = 0.08 #empirical test result john - 20081029



class BaseTest(unittest.TestCase):
    def __init__(self, *args):
        super(BaseTest, self).__init__(*args)
        self.success = False
        self.reached_target_vw = False
        self.duration_start = 0


    def printBaseOdom(self, odom):
        print "odom received"
        print "odom pos " + "x: " + str(odom.pos.x)
        print "odom pos " + "y: " + str(odom.pos.y)
        print "odom pos " + "t: " + str(odom.pos.th)
        print "odom vel " + "x: " + str(odom.vel.x)
        print "odom vel " + "y: " + str(odom.vel.y)
        print "odom vel " + "t: " + str(odom.vel.th)

    def printBaseP3D(self, p3d):
        print "base pose ground truth received"
        print "P3D pose translan: " + "x: " + str(p3d.pos.position.x)
        print "                   " + "y: " + str(p3d.pos.position.y)
        print "                   " + "z: " + str(p3d.pos.position.z)
        print "P3D pose rotation: " + "x: " + str(p3d.pos.orientation.x)
        print "                   " + "y: " + str(p3d.pos.orientation.y)
        print "                   " + "z: " + str(p3d.pos.orientation.z)
        print "                   " + "w: " + str(p3d.pos.orientation.w)
        print "P3D rate translan: " + "x: " + str(p3d.vel.vel.x)
        print "                   " + "y: " + str(p3d.vel.vel.y)
        print "                   " + "z: " + str(p3d.vel.vel.z)
        print "P3D rate rotation: " + "x: " + str(p3d.vel.ang_vel.vx)
        print "                   " + "y: " + str(p3d.vel.ang_vel.vy)
        print "                   " + "z: " + str(p3d.vel.ang_vel.vz)

    def odomInput(self, odom):
        #self.printBaseOdom(odom)
        error = 0


    def p3dInput(self, p3d):
        i = 0
        #self.printBaseP3D(p3d)
        error = abs(p3d.vel.ang_vel.vz - TARGET_VW)
        print " Error: " + str(error)
        # has to reach target vw and maintain target vw for a duration of TARGET_DURATION seconds
        if self.reached_target_vw:
          if error < TARGET_TOL:
            if time.time() - self.duration_start > TARGET_DURATION:
              self.success = True
          else:
            # failed to maintain target vw, reset duration
            self.success = False
            self.reached_target_vw = False
        else:
          if error < TARGET_TOL:
            self.reached_target_vw = True
            self.duration_start = time.time()
    
    def test_base(self):
        print "LNK\n"
        pub = rospy.Publisher("cmd_vel", BaseVel)
        rospy.Subscriber("base_pose_ground_truth", PoseWithRatesStamped, self.p3dInput)
        rospy.Subscriber("odom",                   RobotBase2DOdom,      self.odomInput)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + 60.0
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            pub.publish(BaseVel(0.0,0.0,TARGET_VW))
            time.sleep(0.1)
        self.assert_(self.success)
        
if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], BaseTest, sys.argv) #, text_mode=True)


