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

PKG = 'door_handle_detector' # this package name

import roslib; roslib.load_manifest(PKG) 

import sys
import os
import string

import rospy
from robot_msgs.msg import Point, Point32
from door_msgs.msg import Door
from door_handle_detector.srv import DoorsDetector, DoorsDetectorRequest, DoorsDetectorResponse



def detect_handle(door_request):
    # block until the door_handle_detector service is available
    print "Waiting for service...", rospy.resolve_name('door_handle_vision_detector')
    rospy.wait_for_service('door_handle_vision_detector')
    print "Service is available"
    try:
        find_handle = rospy.ServiceProxy('door_handle_vision_detector', DoorsDetector)
        door_reply = find_handle(door_request)
        print "Request finished"
        return door_reply
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e









if __name__ == "__main__":
    
    door_estimate = Door()
    door_estimate.frame_p1.x = 1.5
    door_estimate.frame_p1.y = 0.5
    door_estimate.frame_p2.x = 1.5
    door_estimate.frame_p2.y = -0.5
    door_estimate.header.frame_id = "odom_combined"

    door_request = DoorsDetectorRequest()
    door_request.door = door_estimate
     # find handle
    door_reply = detect_handle(door_request)

    print door_reply
    print "finished"



    
