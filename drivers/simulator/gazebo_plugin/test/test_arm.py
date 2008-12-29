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

## Gazebo test arm controller
##   sends posision
##   checks to see if P3D returns corresponding ground truth within TARGET_TOL of TARGET_VW
##          for a duration of TARGET_DURATION seconds

PKG = 'gazebo_plugin'
NAME = 'test_arm'

import math
import rostools
rostools.update_path(PKG)
rostools.update_path('rostest')
rostools.update_path('std_msgs')
rostools.update_path('pr2_mechanism_controllers')
rostools.update_path('robot_msgs')
rostools.update_path('rostest')
rostools.update_path('rospy')


import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *
from pr2_mechanism_controllers.msg import *


CMD_POS      =  0.5
TARGET_DURATION = 2.0
TARGET_TOL      = 0.08 #empirical test result john - 20081029

# pre-recorded poses for above commands
TARGET_ARM_TX         =  0.70167519
TARGET_ARM_TY         =  0.27016711
TARGET_ARM_TZ         =  0.78997955
TARGET_ARM_QX         =  0.23727666
TARGET_ARM_QY         = -0.18292768
TARGET_ARM_QZ         =  0.19008174
TARGET_ARM_QW         =  0.93493646
TARGET_GRIPPER_TX     =  0.76451604
TARGET_GRIPPER_TY     =  0.29929084
TARGET_GRIPPER_TZ     =  0.82737631
TARGET_GRIPPER_QX     =  0.22177592
TARGET_GRIPPER_QY     = -0.20146688
TARGET_GRIPPER_QZ     =  0.26480912
TARGET_GRIPPER_QW     =  0.91654743
    

class ArmTest(unittest.TestCase):
    def __init__(self, *args):
        super(ArmTest, self).__init__(*args)
        self.success = False
        self.reached_target_vw = False
        self.duration_start = 0


    def printP3D(self, p3d):
        print "pose ground truth received"
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

    def gripperP3dInput(self, p3d):
        i = 0
        error = abs(p3d.pos.position.x - TARGET_GRIPPER_TX) + \
                abs(p3d.pos.position.y - TARGET_GRIPPER_TY) + \
                abs(p3d.pos.position.z - TARGET_GRIPPER_TZ) + \
                abs(p3d.pos.orientation.x - TARGET_GRIPPER_QX) + \
                abs(p3d.pos.orientation.y - TARGET_GRIPPER_QY) + \
                abs(p3d.pos.orientation.z - TARGET_GRIPPER_QZ) + \
                abs(p3d.pos.orientation.w - TARGET_GRIPPER_QW)
        print " gripper Error: " + str(error)
        #self.printP3D(p3d)
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

    def armP3dInput(self, p3d):
        i = 0
        error = abs(p3d.pos.position.x - TARGET_ARM_TX) + \
                abs(p3d.pos.position.y - TARGET_ARM_TY) + \
                abs(p3d.pos.position.z - TARGET_ARM_TZ) + \
                abs(p3d.pos.orientation.x - TARGET_ARM_QX) + \
                abs(p3d.pos.orientation.y - TARGET_ARM_QY) + \
                abs(p3d.pos.orientation.z - TARGET_ARM_QZ) + \
                abs(p3d.pos.orientation.w - TARGET_ARM_QW)
        print " arm Error: " + str(error)
        #self.printP3D(p3d)
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
    
    def test_arm(self):
        print "LNK\n"
        pub_arm = rospy.Publisher("left_arm_commands", JointPosCmd)
        pub_gripper = rospy.Publisher("l_gripper_controller/set_command", Float64)
        rospy.Subscriber("l_gripper_palm_pose_ground_truth", PoseWithRatesStamped, self.armP3dInput)
        rospy.Subscriber("l_gripper_l_finger_pose_ground_truth", PoseWithRatesStamped, self.gripperP3dInput)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + 30.0
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            pub_arm.publish(JointPosCmd(['l_shoulder_pan_joint','l_shoulder_lift_joint','l_upper_arm_roll_joint','l_elbow_flex_joint','l_forearm_roll_joint','l_wrist_flex_joint','l_wrist_roll_joint'],[CMD_POS,CMD_POS,CMD_POS,CMD_POS,CMD_POS,CMD_POS,CMD_POS],[0,0,0,0,0,0,0],0))
            pub_gripper.publish(Float64(CMD_POS))
            time.sleep(0.1)
        self.assert_(self.success)
        
if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], ArmTest, sys.argv) #, text_mode=True)


