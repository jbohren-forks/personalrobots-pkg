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

PKG = 'test_pr2_mechanism_controllers_gazebo'
NAME = 'test_arm'

import math
import roslib
roslib.load_manifest(PKG)
roslib.load_manifest('rostest')
roslib.load_manifest('numpy')

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *
from pr2_mechanism_controllers.msg import *
from transformations import *
from numpy import *


ARM_JNT_NAMES   = ['l_shoulder_pan_joint','l_shoulder_lift_joint', \
                   'l_upper_arm_roll_joint','l_elbow_flex_joint', \
                   'l_forearm_roll_joint','l_wrist_flex_joint', \
                   'l_wrist_roll_joint']
CMD_POS         = [0.5,0.5,0.5,-0.5,0.5,0.5,0.5]
CMD_VEL         = [0,0,0,0,0,0,0]
GRP_CMD_POS     = 0.3

TARGET_DURATION = 1.0
ROT_TARGET_TOL      = 0.01  #empirical test result john - 20090110
POS_TARGET_TOL      = 0.01  #empirical test result john - 20090110
TEST_TIMEOUT    = 50.0

# pre-recorded poses for above commands
TARGET_ARM_TX         =  0.697079458962
TARGET_ARM_TY         =  0.298321990173
TARGET_ARM_TZ         =  0.788284959682
TARGET_ARM_QX         =  0.237303684388
TARGET_ARM_QY         =  -0.183131139797
TARGET_ARM_QZ         =  0.190090503477
TARGET_ARM_QW         =  0.934887986606
TARGET_GRIPPER_TX     =   0.759604836385
TARGET_GRIPPER_TY     =   0.327703622909
TARGET_GRIPPER_TZ     =   0.825872564018
TARGET_GRIPPER_QX     =   0.201779841484
TARGET_GRIPPER_QY     =   -0.221575348589
TARGET_GRIPPER_QZ     =   0.349992333295
TARGET_GRIPPER_QW     =   0.88752162064

class ArmTest(unittest.TestCase):
    def __init__(self, *args):
        super(ArmTest, self).__init__(*args)
        self.palm_success = False
        self.reached_target_palm = False
        self.duration_start_palm = 0
        self.fngr_success = False
        self.reached_target_fngr = False
        self.duration_start_fngr = 0


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

    def fngrP3dInput(self, p3d):
        i = 0
        pos_error = abs(p3d.pos.position.x - TARGET_GRIPPER_TX) + \
                    abs(p3d.pos.position.y - TARGET_GRIPPER_TY) + \
                    abs(p3d.pos.position.z - TARGET_GRIPPER_TZ)

        #target pose rotation matrix
        target_rm = rotation_matrix_from_quaternion([TARGET_GRIPPER_QX  \
                                                    ,TARGET_GRIPPER_QY  \
                                                    ,TARGET_GRIPPER_QZ  \
                                                    ,TARGET_GRIPPER_QW])

        #p3d pose quaternion
        p3d_q     = [p3d.pos.orientation.x  \
                    ,p3d.pos.orientation.y  \
                    ,p3d.pos.orientation.z  \
                    ,p3d.pos.orientation.w]

        # get error euler angles by inverting the target rotation matrix and multiplying by p3d quaternion
        target_q_inv = quaternion_from_rotation_matrix( linalg.inv(target_rm) )
        rot_euler = euler_from_quaternion( quaternion_multiply(p3d_q, target_q_inv) )
        rot_error = abs( rot_euler[0] ) + \
                    abs( rot_euler[1] ) + \
                    abs( rot_euler[2] )

        print " fngr Error pos: " + str(pos_error) + " rot: " + str(rot_error)

        #self.printP3D(p3d) #for getting new valid data

        # has to reach target vw and maintain target vw for a duration of TARGET_DURATION seconds
        if self.reached_target_fngr:
          print " fngr duration: " + str(time.time() - self.duration_start_fngr)
          if rot_error < ROT_TARGET_TOL and pos_error < POS_TARGET_TOL:
            if time.time() - self.duration_start_fngr > TARGET_DURATION:
              self.fngr_success = True
          else:
            # failed to maintain target vw, reset duration
            self.fngr_success = False
            self.reached_target_fngr = False
        else:
          if rot_error < ROT_TARGET_TOL and pos_error < POS_TARGET_TOL:
            self.reached_target_fngr = True
            self.duration_start_fngr = time.time()

    def palmP3dInput(self, p3d):
        i = 0
        pos_error = abs(p3d.pos.position.x - TARGET_ARM_TX) + \
                    abs(p3d.pos.position.y - TARGET_ARM_TY) + \
                    abs(p3d.pos.position.z - TARGET_ARM_TZ)

        #target pose rotation matrix
        target_rm = rotation_matrix_from_quaternion([TARGET_ARM_QX  \
                                                    ,TARGET_ARM_QY  \
                                                    ,TARGET_ARM_QZ  \
                                                    ,TARGET_ARM_QW])

        #p3d pose quaternion
        p3d_q     = [p3d.pos.orientation.x  \
                    ,p3d.pos.orientation.y  \
                    ,p3d.pos.orientation.z  \
                    ,p3d.pos.orientation.w]

        # get error euler angles by inverting the target rotation matrix and multiplying by p3d quaternion
        target_q_inv = quaternion_from_rotation_matrix( linalg.inv(target_rm) )
        rot_euler = euler_from_quaternion( quaternion_multiply(p3d_q, target_q_inv) )
        rot_error = abs( rot_euler[0] ) + \
                    abs( rot_euler[1] ) + \
                    abs( rot_euler[2] )

        print " palm Error pos: " + str(pos_error) + " rot: " + str(rot_error)

        #self.printP3D(p3d) #for getting new valid data

        # has to reach target vw and maintain target vw for a duration of TARGET_DURATION seconds
        if self.reached_target_palm:
          print " palm duration: " + str(time.time() - self.duration_start_palm)
          if rot_error < ROT_TARGET_TOL and pos_error < POS_TARGET_TOL:
            if time.time() - self.duration_start_palm > TARGET_DURATION:
              self.palm_success = True
          else:
            # failed to maintain target vw, reset duration
            self.palm_success = False
            self.reached_target_palm = False
        else:
          if rot_error < ROT_TARGET_TOL and pos_error < POS_TARGET_TOL:
            self.reached_target_palm = True
            self.duration_start_palm = time.time()
    
    def test_arm(self):
        print "LNK\n"
        pub_arm = rospy.Publisher("left_arm_commands", JointPosCmd)
        pub_gripper = rospy.Publisher("l_gripper_controller/set_command", Float64)
        rospy.Subscriber("l_gripper_palm_pose_ground_truth", PoseWithRatesStamped, self.palmP3dInput)
        rospy.Subscriber("l_gripper_l_finger_pose_ground_truth", PoseWithRatesStamped, self.fngrP3dInput)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + TEST_TIMEOUT
        while not rospy.is_shutdown() and (not self.palm_success or not self.fngr_success) and time.time() < timeout_t:
            pub_arm.publish(JointPosCmd(ARM_JNT_NAMES,CMD_POS,CMD_VEL,0))
            pub_gripper.publish(Float64(GRP_CMD_POS))
            time.sleep(1.0)
        self.assert_(self.palm_success and self.fngr_success)
if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], ArmTest, sys.argv) #, text_mode=True)

