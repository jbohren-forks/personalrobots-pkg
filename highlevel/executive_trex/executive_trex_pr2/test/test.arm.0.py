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
NAME = 'test.arm.0'

import rostools
rostools.update_path(PKG)

import unittest, sys, os, math
import time
import rospy, rostest
from std_msgs.msg import *



import LogReadingTest
        


class TestArm(unittest.TestCase):
    def __init__(self, *args):
        super(TestArm, self).__init__(*args)
        self.test = LogReadingTest.LogReadingTest(rostools.packspec.get_pkg_dir("executive_trex_pr2") + "/test/arm.0.output/latest/TREX.log", 
                                                  [LogReadingTest.LogOnMessage("rightArmState", "ArmState.Holds", [LogReadingTest.RealInterval("shoulder_pan", -1, 0.1)]),
                                                   LogReadingTest.LogOnMessage("rightArmState", "ArmState.Holds", [LogReadingTest.RealInterval("shoulder_pan", 0, 0.1)]),])
        self.test.debugEnable = True
        self.success = False
        
    
    def test_arm(self):
        pub = rospy.TopicPub("initialpose", Pose2DFloat32)
        rospy.ready(NAME, anonymous=True)
        time.sleep(10.0)
        start = Pose2DFloat32()
        start.x = 0
        start.y = 0
        start.th = 0
        pub.publish(start)

        timeout_t = time.time() + 110.0 #90 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            self.test.readLogFile()
            self.success = self.test.getPassed()
            time.sleep(0.2)
        os.system("killall trex_fast")
        os.system("killall gazebo")
        self.assert_(self.success)
        
    


if __name__ == '__main__':
    print "Loading Arm Test..."
    rostest.run(PKG, sys.argv[0], TestArm, sys.argv) #, text_mode=True)
    os.system("killall trex_fast")
    os.system("killall gazebo")




