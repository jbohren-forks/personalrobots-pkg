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
NAME = 'test_hz'

import math
import roslib
roslib.load_manifest(PKG)
roslib.load_manifest('rostest')

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *
from tf.msg import *

MIN_MSGS        = 100
TEST_TIMEOUT    = 100000000000.0

class MyHzTest(unittest.TestCase):
    def __init__(self, *args):
        super(MyHzTest, self).__init__(*args)
        self.tf_success = False
        self.tf_count = 0

        #for time tracking
        self.tf_started = False
        self.tf_start_time = 0
        self.tf_end_time = 0

    def tfInput(self, tf):
        self.tf_count += 1;

        if not self.tf_started:
          self.tf_start_time = rospy.get_rostime().to_seconds()
          print " got tf at: ",self.tf_start_time, " sec"
          self.tf_started = True

        if self.tf_count >= MIN_MSGS:
          self.tf_end_time = rospy.get_rostime().to_seconds()
          print " got ",self.tf_count," tf messages at ",self.tf_count / (self.tf_end_time - self.tf_start_time), " Hz"
          self.tf_success = True
    
        print " got ",self.tf_count," tf messages at ",self.tf_count / (rospy.get_rostime().to_seconds() - self.tf_start_time), " Hz"

    def test_hz(self):
        print "LNK\n"
        rospy.Subscriber("/tf_message", tfMessage, self.tfInput)
        rospy.init_node(NAME, anonymous=True)
        timeout_t = time.time() + TEST_TIMEOUT
        while not rospy.is_shutdown() and not self.tf_success and time.time() < timeout_t:
            time.sleep(1.0)

        self.assert_(self.tf_success)

if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], MyHzTest, sys.argv) #, text_mode=True)

