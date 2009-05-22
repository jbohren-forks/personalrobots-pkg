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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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
# Revision $Id: add_two_ints_client 3804 2009-02-11 02:16:00Z rob_wheeler $

## Gets a pointcloud from a bag file, and calls the handle detector

PKG = 'outlet_detection' # this package name

import roslib; roslib.load_manifest(PKG) 

import sys
import os
import string
import rospy

import rostest
import unittest
import math

# imports the handle detector service
from outlet_detection.srv import *
from robot_msgs.msg import PointStamped, PoseStamped, Quaternion

class TestOutletDetector(unittest.TestCase):
    def setUp(self):
        rospy.init_node("handle_tester")

         # Threshold for agreement between laser and camera detection
        self.dist_epsilon = 0.1  # 10 cm
        self.angle_epsilon = 10*math.pi/180  # 10 degrees

    def multiply_quaternions(self,q1,q2):        
        q = Quaternion()
        q.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z
        q.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y
        q.y = q1.w*q2.y + q1.y*q2.w + q1.z*q2.x - q1.x*q2.z
        q.z = q1.w*q2.z + q1.z*q2.w + q1.x*q2.y - q1.y*q2.x

        return q

    def conjugate(self,q):
        r = Quaternion()
        r.w = q.w
        r.x = -q.x
        r.y = -q.y
        r.z = -q.z
        return r

    def point_dist(self,p1,p2):
        dx = p1.x-p2.x
        dy = p1.y-p2.y
        dz = p1.z-p2.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)


    def angle(self,p1,p2):
        return math.acos(p1.x*p2.x+p1.y*p2.y+p1.z*p2.z)

    def quaternion_angle(self,q1,q2):
        # rotate the same vector by teh two quaternions
        # and measure angle
        v = Quaternion(1,0,0,0)
        v1 = self.multiply_quaternions(self.multiply_quaternions(q1,v),self.conjugate(q1))
        v2 = self.multiply_quaternions(self.multiply_quaternions(q2,v),self.conjugate(q2))

        return self.angle(v1,v2)
			

    def check_poses(self, pose, gt_pose):
         p1 = pose.pose.position
         p2 = gt_pose.pose.position

         distance = self.point_dist(p1,p2)
         print "Distance: ",distance*100,"cm"

         self.assertTrue(distance<self.dist_epsilon)

         q1 = pose.pose.orientation
         q2 = gt_pose.pose.orientation
         angle = self.quaternion_angle(q1,q2)
         print "Angle", angle*180/math.pi, "degrees"

         self.assertTrue(angle<self.angle_epsilon)

    def test_outlet_detector(self):
        rospy.sleep(1.0)
        p = PointStamped()
        p.point.x = 1
        p.point.x = 0
        p.point.x = 0
        p.header.frame_id = "base_footprint"
        p.header.stamp = rospy.get_rostime()
        pose = self.detect_outlet(p)

        if len(sys.argv)<2:
            self.fail("No ground truth present")

        if sys.argv[1]=='fail':
            self.assertTrue(pose==None)
            return

        filename = sys.argv[1]
        print sys.argv
        if os.path.exists(filename) and os.path.isfile(filename):
            if pose==None:
                #fail test
                self.assertTrue(False)
            file = open(filename,"r")
            gt_pose = PoseStamped()
            gt_pose.deserialize(file.read())
            file.close()
            self.check_poses(pose, gt_pose)
        else:
            if len(sys.argv)>=3 and sys.argv[2]=='save':
                file = open(filename,"w")
                pose.serialize(file)
                file.close()
            # failing test
            self.assertTrue(False)
         

    def detect_outlet(self, point):
         # block until the door_handle_detector service is available
        service_name = 'outlet_spotting/coarse_outlet_detect'
        print "Waiting for service...", rospy.resolve_name(service_name)
        rospy.wait_for_service(service_name)
        try:
            print "Getting service proxy"
            find_outlet = rospy.ServiceProxy(service_name, OutletDetection)
            print "Calling service"
            resp = find_outlet(point)
            print "Request finished"
            print "Outlet detected at ",resp.pose
            return resp.pose
        except:
            return None


if __name__ == "__main__":
    rostest.run('outlet_detection', 'outlet_spotting_test', 
            TestOutletDetector, sys.argv)
