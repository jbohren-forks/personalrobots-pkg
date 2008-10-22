#!/usr/bin/env python
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Willow Garage, Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Melonee Wise

PKG = "pr2_mechanism_controllers"

import rostools; rostools.update_path(PKG)
#rostools.update_path('std_msgs')

import sys
import os
import string
from time import sleep

import rospy
from std_msgs.msg import PointStamped, Point
from pr2_mechanism_controllers.srv import SetJointCmd, SetJointCmdRequest

def point_head_client(pan, tilt):
    
      # NOTE: you don't have to call rospy.init_node() to make calls against
    # a service. This is because service clients do not have to be
    # nodes.

    # block until the add_two_ints service is available
    print "waiting for service"
    rospy.wait_for_service('head_controller/set_command_array')
    
    try:
        # create a handle to the add_two_ints service
        send_cmd= rospy.ServiceProxy('head_controller/set_command_array', SetJointCmd)
        
        print "Requesting %f,%f"%(pan, tilt)
        
        # simplified style
        resp2 = send_cmd.call(SetJointCmdRequest([pan, tilt],[0.0, 0.0],[0.0, 0.0],['head_pan_joint', 'head_tilt_joint']))
        print resp2.positions, resp2.names
        return resp2
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

       

def point_head_cart_client(x,y,z,frame):

    
    head_angles = rospy.Publisher('head_controller/track_point', PointStamped)
    rospy.init_node('head_commander', anonymous=True)
    sleep(1)
    head_angles.publish(PointStamped(rostools.msg.Header(None, None, frame), Point(x, y, z)))



def usage():
    return "%s [pan tilt] or [x,y,z,frame]"%sys.argv[0]

if __name__ == "__main__":
    
    if len(sys.argv) < 3:
        print usage()
        sys.exit(1)
    elif len(sys.argv) ==3:
        point_head_client(float(sys.argv[1]), float(sys.argv[2]))
    elif len(sys.argv) ==5:
        point_head_cart_client(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), sys.argv[4])
