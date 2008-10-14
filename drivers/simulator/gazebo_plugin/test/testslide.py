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

## Gazebo collision validation 

NAME = 'testslide'

import rostools
rostools.update_path('gazebo_plugin')
rostools.update_path('std_msgs')
rostools.update_path('robot_msgs')
rostools.update_path('rostest')
rostools.update_path('rospy')

import unittest, sys, os, math
import time
import rospy, rostest
from std_msgs.msg import *
from robot_msgs.msg import *

TARGET_X = -5.4 + 25.65 #contains offset specified in P3D for base, alternatively, use the gripper roll ground truths
TARGET_Y = 0.0 + 25.65 #contains offset specified in P3D for base, alternatively, use the gripper roll ground truths
TARGET_Z = 3.8
TARGET_RAD = 4.5

class TestSlide(unittest.TestCase):
    def __init__(self, *args):
        super(TestSlide, self).__init__(*args)
        self.success = False
        self.fail = False
        self.hits = 0
        self.runs = 0
        
    def positionInput(self, pose):
        self.runs = self.runs + 1
        print " got pose ", self.runs
        #if (pos.frame == 1):
        print "x ", pose.transform.translation.x
        print "y ", pose.transform.translation.y
        print "z ", pose.transform.translation.z
        dx = pose.transform.translation.x - TARGET_X
        dy = pose.transform.translation.y - TARGET_Y
        dz = pose.transform.translation.z - TARGET_Z
        d = math.sqrt((dx * dx) + (dy * dy)) #+ (dz * dz))
        print "P: " + str(pose.transform.translation.x) + " " + str(pose.transform.translation.y)
        #print "D: " + str(dx) + " " + str(dy) + " " + str(dz) + " " + str(d) + " < " + str(TARGET_RAD * TARGET_RAD)
        if (d < TARGET_RAD):
            #print "HP: " + str(dx) + " " + str(dy) + " " + str(d) + " at " + str(pos.transform.translation.x) + " " + str(pos.transform.translation.y)
            #print "DONE"
            self.hits = self.hits + 1
            print "Hit goal, " + str(self.hits)
            if (self.runs < 100 and self.runs > 10):
                print "Obviously wrong transforms!"
                self.success = False
                self.fail = True
                #os.system("killall gazebo")
                
            if (self.hits > 200):
                if (self.runs > 20):
                    self.success = True
                #os.system("killall gazebo")
        
    
    def testslide(self):
        print "LINK\n"
        #rospy.Subscriber("Odom", RobotBase2DOdom, self.positionInput)
        rospy.Subscriber("base_pose_ground_truth", TransformWithRateStamped, self.positionInput)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + 50.0 #59 seconds
        while not rospy.is_shutdown() and not self.success and not self.fail and time.time() < timeout_t:
            time.sleep(0.1)
        time.sleep(2.0)
        #os.system("killall gazebo")
        self.assert_(self.success)
        
    


if __name__ == '__main__':
    rostest.run('gazebo_plugin', sys.argv[0], TestSlide, sys.argv) #, text_mode=True)
    #os.system("killall gazebo")


