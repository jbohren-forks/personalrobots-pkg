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

## Gazebo test 2dnav stack

PKG = '2dnav_gazebo'
NAME = 'test_2dnav_translations'

import math
import rostools
rostools.update_path(PKG)
rostools.update_path('rostest')
rostools.update_path('std_msgs')
rostools.update_path('robot_msgs')
rostools.update_path('rostools')
rostools.update_path('rospy')
rostools.update_path('transformations')


import sys, unittest
import os, os.path, threading, time
import rospy, rostest, rostools
from std_msgs.msg import *
from rostools.msg import *
from transformations import *
from numpy import *


class NavStackTest(unittest.TestCase):
    def __init__(self, *args):
        super(NavStackTest, self).__init__(*args)
        self.bumped  = False
        self.success = False

        self.odom_xi = 0
        self.odom_yi = 0
        self.odom_qi = [0,0,0,0]
        self.odom_initialized = False;

        self.odom_x = 0
        self.odom_y = 0
        self.odom_q = [0,0,0,0]

        self.p3d_xi = 0
        self.p3d_yi = 0
        self.p3d_qi = [0,0,0,0]
        self.p3d_initialized = False;

        self.p3d_x = 0
        self.p3d_y = 0
        self.p3d_q = [0,0,0,0]

        # parameters
        self.nav_tol      = 0.1
        self.odom_tol     = 1.0
        self.test_timeout = 50.0

        # starting position of the robot is 25.65, 25.65 (center of map)
        # goal position
        self.target_x =  25.65
        self.target_y =  25.65
        self.target_t =  0.0

        self.args = sys.argv
        

    def printBaseOdom(self, odom):
        print "odom received"
        print "odom pos " + "x: " + str(odom.pos.x)
        print "odom pos " + "y: " + str(odom.pos.y)
        print "odom pos " + "t: " + str(odom.pos.th)
        print "odom vel " + "x: " + str(odom.vel.x)
        print "odom vel " + "y: " + str(odom.vel.y)
        print "odom vel " + "t: " + str(odom.vel.th)

    def printBaseP3D(self, p3d):
        print "base pose ground truth received"
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



    def odomInput(self, odom):
        #self.printBaseOdom(odom)
        # initialize odom
        if self.odom_initialized == False or self.p3d_initialized == False:
            self.odom_initialized = True
            self.odom_xi = odom.pos.x
            self.odom_yi = odom.pos.y
            self.odom_qi = quaternion_from_euler(0,0,odom.pos.th,'rxyz')
        else:
            # update odom
            self.odom_x = odom.pos.x
            self.odom_y = odom.pos.y
            self.odom_q = quaternion_from_euler(0,0,odom.pos.th,'rxyz')

    def p3dInput(self, p3d):
        #self.printBaseP3D(p3d)
        # initialize ground truth
        if self.odom_initialized == False or self.p3d_initialized == False:
            self.p3d_initialized = True
            self.p3d_xi = p3d.pos.position.x
            self.p3d_yi = p3d.pos.position.y
            self.p3d_qi =[ p3d.pos.orientation.x \
                          ,p3d.pos.orientation.y \
                          ,p3d.pos.orientation.z \
                          ,p3d.pos.orientation.w]
        else:
            # update ground truth
            self.p3d_x = p3d.pos.position.x
            self.p3d_y = p3d.pos.position.y
            self.p3d_q =[ p3d.pos.orientation.x \
                         ,p3d.pos.orientation.y \
                         ,p3d.pos.orientation.z \
                         ,p3d.pos.orientation.w]

    def bumpedInput(self, bumpString):
        print "robot touched something! ", bumpString.data
        self.bumped = True
    
    def test_2dnav_translations(self):
        print "LNK\n"
        #pub_base = rospy.Publisher("cmd_vel", BaseVel)
        pub_goal = rospy.Publisher("goal", Planner2DGoal) #received by wavefront_player or equivalent
        rospy.Subscriber("base_pose_ground_truth", PoseWithRatesStamped, self.p3dInput)
        rospy.Subscriber("odom"                  , RobotBase2DOdom     , self.odomInput)
        rospy.Subscriber("base_bumper"           , String              , self.bumpedInput)
        rospy.Subscriber("torso_lift_bumper"     , String              , self.bumpedInput)

        rospy.init_node(NAME, anonymous=True)

        timeout_t = time.time() + self.test_timeout

        # get goal from commandline
        print "------------------------"
        for i in range(0,len(self.args)):
          print " sys argv:", self.args[i]
          if self.args[i] == '-x':
            if len(self.args) > i+1:
              self.target_x = float(self.args[i+1])
              print "target x set to:",self.target_x
          if self.args[i] == '-y':
            if len(self.args) > i+1:
              self.target_y = float(self.args[i+1])
              print "target y set to:",self.target_y
          if self.args[i] == '-t':
            if len(self.args) > i+1:
              self.target_t = float(self.args[i+1])
              self.target_q =  quaternion_from_euler(0,0,self.target_t,'rxyz')
              print "target t set to:",self.target_t
          if self.args[i] == '-nav_tol':
            if len(self.args) > i+1:
              self.nav_tol = float(self.args[i+1])
              print "nav_tol set to:",self.nav_tol
          if self.args[i] == '-odom_tol':
            if len(self.args) > i+1:
              self.odom_tol = float(self.args[i+1])
              print "odom_tol set to:",self.odom_tol
        print " target:", self.target_x, self.target_y, self.target_t
        print "------------------------"
        # wait for result
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            # send goal
            h = Header();
            h.stamp = rospy.get_rostime();
            h.frame_id = "map"
            pub_goal.publish(Planner2DGoal(h,Pose2DFloat32(self.target_x,self.target_y,self.target_t),1))
            time.sleep(2.0)
            # compute angular error between deltas in odom and p3d
            # compute delta in odom from initial pose
            print "========================"
            tmpori = rotation_matrix_from_quaternion(self.odom_qi)
            tmpoqi = quaternion_from_rotation_matrix(linalg.inv(tmpori))
            odom_q_delta = quaternion_multiply(tmpoqi,self.odom_q)
            print "odom delta:" , euler_from_quaternion(odom_q_delta)
            # compute delta in p3d from initial pose
            tmppri = rotation_matrix_from_quaternion(self.p3d_qi)
            tmppqi = quaternion_from_rotation_matrix(linalg.inv(tmppri))
            p3d_q_delta = quaternion_multiply(tmppqi,self.p3d_q)
            print "p3d delta:" , euler_from_quaternion(p3d_q_delta)
            # compute delta between odom and p3d
            tmpdri = rotation_matrix_from_quaternion(p3d_q_delta)
            tmpdqi = quaternion_from_rotation_matrix(linalg.inv(tmpdri))
            delta = quaternion_multiply(tmpdqi,odom_q_delta)
            delta_euler = euler_from_quaternion(delta)
            odom_drift_dyaw = delta_euler[2]
            print "odom drift from p3d:" , euler_from_quaternion(delta)

            # compute delta between target and p3d
            tmptri = rotation_matrix_from_quaternion(self.target_q)
            tmptqi = quaternion_from_rotation_matrix(linalg.inv(tmptri))
            navdq = quaternion_multiply(tmptqi,self.p3d_q)
            navde = euler_from_quaternion(navdq)
            nav_dyaw = navde[2]
            print "nav euler off target:" , navde

            # check odom error (odom error from ground truth)
            odom_error =  abs(self.odom_x - self.p3d_x - self.odom_xi + self.p3d_xi ) \
                        + abs(self.odom_y - self.p3d_y - self.odom_yi + self.p3d_yi ) \
                        + abs(odom_drift_dyaw)

            # check total error (difference between ground truth and target)
            nav_error  =  abs(self.p3d_x - self.target_x) \
                        + abs(self.p3d_y - self.target_y) \
                        + abs(nav_dyaw)
            print "nav error:" + str(nav_error) + " nav_tol:" + str(self.nav_tol) + " odom error:" + str(odom_error) + " odom_tol: " + str(self.odom_tol)

            if nav_error < self.nav_tol and self.bumped == False and odom_error < self.odom_tol:
                self.success = True

        self.assert_(self.success)
        
if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], NavStackTest, sys.argv) #, text_mode=True)


