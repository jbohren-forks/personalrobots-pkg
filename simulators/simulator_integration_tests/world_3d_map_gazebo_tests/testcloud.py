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

## Gazebo test cameras validation 

PKG = 'simulator_integration_tests'
NAME = 'testcloud'

import rostools
rostools.update_path(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *



class CloudTest(unittest.TestCase):
    def __init__(self, *args):
        super(CloudTest, self).__init__(*args)
        self.success = False
        self.errorcheck = False


    def pointInput(self, cloud):
        print "Input " + str(len(cloud.pts)) + " points."
        avg = 0
        for pt in cloud.pts:
            avg = avg + pt.x
        avg = avg / len(cloud.pts)
        
        variance = 0
        for pt in cloud.pts:
            variance = (pt.x - avg) * (pt.x - avg)
        variance = variance / len(cloud.pts)
            
        print "Mean: " + str(avg)
        print "Variance: " + str(variance)
        self.errorcheck = (avg > 8.9 and avg < 9.1 and variance < 0.2)
    
    def test_cloud(self):
        print "LNK\n"
        rospy.TopicSub("world_3d_map", PointCloud, self.pointInput)
        rospy.ready(NAME, anonymous=True)
        timeout_t = time.time() + 30.0 #30 seconds
        while not rospy.is_shutdown() and not self.success and time.time() < timeout_t:
            time.sleep(0.1)
        self.success = self.errorcheck
        os.system("killall gazebo")
        self.assert_(self.success)
        
    


if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], CloudTest, sys.argv) #, text_mode=True)


