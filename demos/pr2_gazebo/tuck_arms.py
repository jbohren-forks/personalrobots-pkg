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

## Gazebo tug arms for navigation

PKG = 'pr2_gazebo'
NAME = 'tuck_arms'

import math
import rostools
rostools.update_path(PKG)
rostools.update_path('rostest')
rostools.update_path('std_msgs')
rostools.update_path('pr2_mechanism_controllers')
rostools.update_path('robot_msgs')
rostools.update_path('rospy')


import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *
from pr2_mechanism_controllers.msg import *


CMD_POS_1      =  0.0
CMD_POS_2      =  3.142
CMD_POS_3      =  0.0
CMD_POS_4      =  3.142
CMD_POS_5      =  3.142
CMD_POS_6      =  3.142
CMD_POS_7      =  0.0
CMD_POS_8      =  0.8

if __name__ == '__main__':
    pub_l_arm = rospy.Publisher("left_arm_commands", JointPosCmd)
    pub_l_gripper = rospy.Publisher("l_gripper_controller/set_command", Float64)
    pub_r_arm = rospy.Publisher("right_arm_commands", JointPosCmd)
    pub_r_gripper = rospy.Publisher("r_gripper_controller/set_command", Float64)
    rospy.init_node(NAME, anonymous=True)
    timeout_t = time.time() + 10.0 #publish for 10 seconds then stop
    while time.time() < timeout_t:
        pub_l_arm.publish(JointPosCmd(['l_shoulder_pan_joint','l_shoulder_lift_joint','l_upper_arm_roll_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint'],[CMD_POS_1,CMD_POS_2,CMD_POS_3,CMD_POS_4,CMD_POS_5,CMD_POS_6,CMD_POS_7],[0,0,0,0,0,0,0],0))
        pub_l_gripper.publish(Float64(CMD_POS_8))
        pub_r_arm.publish(JointPosCmd(['r_shoulder_pan_joint','r_shoulder_lift_joint','r_upper_arm_rolr_joint','r_elbow_flex_joint','r_forearm_roll_joint','r_wrist_flex_joint','r_wrist_roll_joint'],[CMD_POS_1,CMD_POS_2,CMD_POS_3,CMD_POS_4,CMD_POS_5,CMD_POS_6,CMD_POS_7],[0,0,0,0,0,0,0],0))
        pub_r_gripper.publish(Float64(CMD_POS_8))
        time.sleep(0.2)

