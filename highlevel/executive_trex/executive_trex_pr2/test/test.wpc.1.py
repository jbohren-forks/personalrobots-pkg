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

## Trex demo validation.

PKG = 'executive_trex_pr2'
NAME = 'test.wpc.1'

import rostools
rostools.update_path(PKG)

import unittest, sys, os, math
import time
import rospy, rostest
from std_msgs.msg import *

ERROR_TOL = [1.0, 1.0]
TARGETS = [[20.0, 20.0], [23.0, 15.0], [21.5, 11.5]]
START_X = 22.0
START_Y = 12.0
START_TH = 0

import WpcControllerTest
        


class TestWpc(unittest.TestCase):
    def __init__(self, *args):
        super(TestWpc, self).__init__(*args)
        self.test = WpcControllerTest.WpcControllerTest(TARGETS, ERROR_TOL, [START_X, START_Y, START_TH])
        self.success = False
        
    def positionInput(self, pos):
        self.test.positionInput2d(pos)
        if (self.test.success()):
            self.success = True
    
    def test_wpc(self):
        rospy.TopicSub("groundtruthposition", Point32, self.positionInput)
        self.test.advertiseInitalPose2d(NAME)
        rospy.ready(NAME, anonymous=True)
        time.sleep(2.0)
        self.test.publishInitalPose2d(NAME)

        timeout_t = time.time() + 50.0 #59 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        time.sleep(2.0)
        os.system("killall trex_fast")
        self.assert_(self.success)
        
    


if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], TestWpc, sys.argv) #, text_mode=True)
    os.system("killall trex_fast")




