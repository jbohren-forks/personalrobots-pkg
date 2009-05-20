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

PKG = 'door_handle_detector' # this package name

import roslib; roslib.load_manifest(PKG) 

import sys
import os
import string
from rosrecord import *
import rospy

# imports the handle detector service
from door_handle_detector.srv import *
from door_msgs.msg import *



def detect_door_laser(door_request):
    # block until the door_handle_detector service is available
    print "Waiting for service...", rospy.resolve_name('doors_detector')
    rospy.wait_for_service('doors_detector')
    try:
        print "Getting service proxy"
        find_door_laser = rospy.ServiceProxy('doors_detector', DoorsDetector)
        print "Calling service"
        door_reply = find_door_laser(door_request)
        print "Request finished"
        print "Door detected by laser at (%f, %f) (%f, %f)"%(door_reply.doors[0].door_p1.x, door_reply.doors[0].door_p1.y, door_reply.doors[0].door_p2.x, door_reply.doors[0].door_p2.y)
        return door_reply
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e



def detect_handle_laser(door_request):
    # block until the door_handle_detector service is available
    print "Waiting for service...", rospy.resolve_name('handle_detector')
    rospy.wait_for_service('doors_detector')
    print "Service is available"
    try:
        print "Getting service proxy"
        find_handle_laser = rospy.ServiceProxy('handle_detector', DoorsDetector)
        print "Calling service"
        door_reply = find_handle_laser(door_request)
        print "Request finished"
        print "Handle detected by laser at (%f, %f, %f)"%(door_reply.doors[0].handle.x, door_reply.doors[0].handle.y, door_reply.doors[0].handle.z)
        return door_reply
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e





def detect_handle_camera(door_request):
    # block until the door_handle_detector service is available
    print "Waiting for service...", rospy.resolve_name('door_handle_vision_detector')
    rospy.wait_for_service('door_handle_vision_detector')
    print "Service is available"
    try:
        print "Getting service proxy"
        find_handle = rospy.ServiceProxy('door_handle_vision_detector', DoorsDetector)
        print "Calling service"
        door_reply = find_handle(door_request)
        print "Request finished"
        print "Handle detected by camera at (%f, %f, %f)"%(door_reply.doors[0].handle.x, door_reply.doors[0].handle.y, door_reply.doors[0].handle.z)
        return door_reply
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e





if __name__ == "__main__":
    rospy.init_node("handle_tester")
    
    rospy.sleep(2.0)
    d = Door()
    d.frame_p1.x = 1.0
    d.frame_p1.y = -0.5
    d.frame_p2.x = 1.0
    d.frame_p2.y = 0.5
    d.hinge = d.HINGE_P2
    d.rot_dir = d.ROT_DIR_COUNTERCLOCKWISE
    d.header.frame_id = "base_footprint"
    d.header.stamp = rospy.get_rostime()
    print "time ",d.header.stamp
    

    resp = detect_door_laser(d)
    resp = detect_handle_laser(resp.doors[0])
    resp = detect_handle_camera(d)    
