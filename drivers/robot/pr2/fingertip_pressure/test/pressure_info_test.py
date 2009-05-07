#!/usr/bin/env python
#***********************************************************
#* Software License Agreement (BSD License)
#*
#*  Copyright (c) 2008, Willow Garage, Inc.
#*  All rights reserved.
#*
#*  Redistribution and use in source and binary forms, with or without
#*  modification, are permitted provided that the following conditions
#*  are met:
#*
#*   * Redistributions of source code must retain the above copyright
#*     notice, this list of conditions and the following disclaimer.
#*   * Redistributions in binary form must reproduce the above
#*     copyright notice, this list of conditions and the following
#*     disclaimer in the documentation and/or other materials provided
#*     with the distribution.
#*   * Neither the name of the Willow Garage nor the names of its
#*     contributors may be used to endorse or promote products derived
#*     from this software without specific prior written permission.
#*
#*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#*  POSSIBILITY OF SUCH DAMAGE.
#***********************************************************

# Author: Blaise Gassend

import roslib; roslib.load_manifest('fingertip_pressure')

import rospy
import sys
import time
import unittest
import rostest

from fingertip_pressure.msg import PressureInfo

## A sample python unit test
class PressureInfoTest(unittest.TestCase):
    def __init__(self, *args):
        super(PressureInfoTest, self).__init__(*args)
        rospy.init_node('pressure_info_test')
        
        # Read one message from the topic
        self.msg = None
        sub = rospy.Subscriber('pressure/r_gripper_motor_info',
                PressureInfo, self.callback)
        timeout_t = rospy.get_time() + 2
        print 'waiting for message'
        while self.msg == None and timeout_t > rospy.get_time():
            rospy.sleep(0.1)
        print 'done waiting for message'
        sub.unregister()
        self.assertNotEquals(self.msg, None)
                                    
    def callback(self, msg):
        print 'got message'
        self.msg = msg

    def test_array_sizes(self):
        self.assertEquals(len(self.msg.sensor), 2)
        self.assertEquals(len(self.msg.sensor[1].center), 22)
        self.assertEquals(len(self.msg.sensor[1].halfside1), 22)
        self.assertEquals(len(self.msg.sensor[1].halfside2), 22)

if __name__ == '__main__':
    import rostest
    time.sleep(0.75)
    try:
        rostest.rosrun('fingertip_pressure', 'pressure_info_test', PressureInfoTest)
    except KeyboardInterrupt, e:
        pass
    print "exiting"



