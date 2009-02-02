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
# Revision $Id: add_two_ints_client 3357 2009-01-13 07:13:05Z jfaustwg $

## Simple demo of a rospy service client that calls a service to add
## two integers. 

PKG = 'rospy_tutorials' # this package name

import rostools; rostools.update_path(PKG) 

import sys
import os
import string

import rospy
from std_msgs.msg import Point

def find_door(p1, p2):
    # block until the door_handle_detector service is available
    print "Waiting for service...", rospy.resolve_name('door_handle_detector')
    rospy.wait_for_service('door_handle_detector')
    print "Service is available"
    try:
        # create a handle to the add_two_ints service
        find_door = rospy.ServiceProxy('door_handle_detector', FindDoor)
        print "Requesting (%s, %s) and (%s, %s)"%(p1.x, p1.y, p2.x, p2.y)
        res = find_door(p1, p2)
        return res
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e





def move_arm(x, y, z, rx, ry, rz, w):
    print "Create publisher to arm"
    pub = rospy.Publisher('/arm_pose/command', PoseStamped)
    
    m = PoseStamped()
    m.header.frame_id = 'odom_combined'
    m.pose.position.x = x
    m.pose.position.y = y
    m.pose.position.z = z
    m.pose.orientation.x = rx
    m.pose.orientation.y = ry
    m.pose.orientation.z = rz
    m.pose.orientation.w = w

    print "Publish pose command to arm"
    pub.publish(m)
    rospy.init_node('pub', anonymous=True)





if __name__ == "__main__":
    
    p1 = Point
    p2 = Point

    p1.x = 1.0
    p1.y = -0.5
    p2.x = 1.0
    p2.y = 0.5
    
    res = find_door(p1, p2)
    print "Frame found at (%s, %s)  (%s, %s)"%(res.frame_p1.x, res.frame_p1.y, 
                                               res.frame_p2.x, res.frame_p2.y)
    print "Door found at (%s, %s)  (%s, %s)"%(res.door_p1.x, res.door_p1.y, 
                                              res.door_p2.x, res.door_p2.y)
    print "Handle found at (%s, %s)"%(res.handle.x, res.handle.y)


    
