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

PKG = 'test_pr2_mechanism_controllers_gazebo'
NAME = 'test_base_vw_gt'

import math
import roslib
roslib.load_manifest(PKG)
roslib.load_manifest('rostest')

import sys, unittest
import os, time
import rospy, rostest
from geometry_msgs.msg import Twist,Vector3
from nav_msgs.msg import Odometry

TEST_DURATION   = 60.0

TARGET_VW       =  0.5
TARGET_DURATION = 2.0
TARGET_TOL      = 0.15 #empirical test result john - 20090420



class BaseTest(unittest.TestCase):
    def __init__(self, *args):
        super(BaseTest, self).__init__(*args)
        self.success = False
        self.reached_target_vw = False
        self.duration_start = 0


    def printBaseOdom(self, odom):
        orientation = odom.pose.orientation
        q = Q(orientation.x, orientation.y, orientation.z, orientation.w)
        q.normalize()
        print "odom received"
        print "odom pos " + "x: " + str(odom.pose.pose.position.x)
        print "odom pos " + "y: " + str(odom.pose.pose.position.y)
        print "odom pos " + "t: " + str(q.getEuler().z)
        print "odom vel " + "x: " + str(odom.twist.twist.linear.x)
        print "odom vel " + "y: " + str(odom.twist.twist.linear.y)
        print "odom vel " + "t: " + str(odom.twist.twist.angular.z)

    def printBaseP3D(self, p3d):
        print "base pose ground truth received"
        print "P3D pose translan: " + "x: " + str(p3d.pose.pose.position.x)
        print "                   " + "y: " + str(p3d.pose.pose.position.y)
        print "                   " + "z: " + str(p3d.pose.pose.position.z)
        print "P3D pose rotation: " + "x: " + str(p3d.pose.pose.orientation.x)
        print "                   " + "y: " + str(p3d.pose.pose.orientation.y)
        print "                   " + "z: " + str(p3d.pose.pose.orientation.z)
        print "                   " + "w: " + str(p3d.pose.pose.orientation.w)
        print "P3D rate translan: " + "x: " + str(p3d.twist.twist.linear.x)
        print "                   " + "y: " + str(p3d.twist.twist.linear.y)
        print "                   " + "z: " + str(p3d.twist.twist.linear.z)
        print "P3D rate rotation: " + "x: " + str(p3d.twist.twist.angular.x)
        print "                   " + "y: " + str(p3d.twist.twist.angular.y)
        print "                   " + "z: " + str(p3d.twist.twist.angular.z)

    def odomInput(self, odom):
        #self.printBaseOdom(odom)
        error = 0


    def p3dInput(self, p3d):
        i = 0
        #self.printBaseP3D(p3d)
        error = abs(p3d.twist.twist.angular.z - TARGET_VW)
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
        pub = rospy.Publisher("/cmd_vel", Twist)
        rospy.Subscriber("/base_pose_ground_truth", Odometry, self.p3dInput)
        rospy.Subscriber("/odom",                   Odometry, self.odomInput)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + TEST_DURATION
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            pub.publish(Twist(Vector3(0.0,0.0,0), Vector3(0,0,TARGET_VW)))
            time.sleep(0.1)
        self.assert_(self.success)
        
if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], BaseTest, sys.argv) #, text_mode=True)


