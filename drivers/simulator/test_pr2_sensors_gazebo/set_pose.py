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

## Set head and gripper poses

PKG = 'test_pr2_sensors_gazebo'
NAME = 'set_pose'

import math
import roslib
roslib.load_manifest(PKG)
roslib.load_manifest('rostest')


import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *
from pr2_mechanism_controllers.msg import *


CMD_POS_1      =  0.02
CMD_POS_2      = -0.2
CMD_POS_3      =  0.2

if __name__ == '__main__':
    pub_l_gripper = rospy.Publisher("/l_gripper_controller/set_command", Float64)
    pub_r_gripper = rospy.Publisher("/r_gripper_controller/set_command", Float64)
    pub_head_pan  = rospy.Publisher("/head_pan_controller/set_command", Float64)
    pub_head_tilt = rospy.Publisher("/head_tilt_controller/set_command", Float64)
    rospy.init_node(NAME, anonymous=True)
    timeout_t = time.time() + 20.0 #publish for 20 seconds then stop
    while time.time() < timeout_t:
        pub_l_gripper.publish(Float64(CMD_POS_1))
        pub_r_gripper.publish(Float64(CMD_POS_1))
        pub_head_pan.publish(Float64(CMD_POS_2))
        pub_head_tilt.publish(Float64(CMD_POS_3))
        time.sleep(0.2)

