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
# Revision $Id: test_embed_msg.py 1986 2008-08-26 23:57:56Z sfkwc $

## Integration test for empty services to test serializers
## and transport

PKG = 'mechanism_control'
NAME = 'test_ms'
PUBTOPIC = "chatter"
PUBNODE = 'mechanism_control'
SUBTOPIC = 'mechanism_state'

import rostools; rostools.update_path(PKG)

import sys, time
import unittest

# ad-hoc import here as we don't want to change the manifest for test dependencies
rostools.update_path('rostest')
import rospy, rostest
from mechanism_control.msg import MechanismState, ActuatorState, JointState

MSG = MechanismState

TIMEOUT = 10.0 #seconds

class TestMechanismState(unittest.TestCase):

    def setUp(self):
        self.callback_data = None
        
    def _test_ms_callback(self, data):
        print "GOT CALLBACK DATA"
        self.callback_data = data
    
    def test_ms_msg(self):
        self.assert_(self.callback_data is None, "invalid test fixture")

        if 0:
            # wait at most 5 seconds for publisher to be registered
            timeout_t = time.time() + 5.0
            while not rostest.is_subscriber(
                rospy.resolve_name(PUBTOPIC),
                rospy.resolve_name(PUBNODE)) and time.time() < timeout_t:
                time.sleep(0.1)

            self.assert_(rostest.is_subscriber(
                rospy.resolve_name(PUBTOPIC),
                rospy.resolve_name(PUBNODE)), "%s is not up"%PUBNODE)
        
        print "Subscribing to ", SUBTOPIC
        rospy.TopicSub(SUBTOPIC, MSG, self._test_ms_callback) 

        print "Waiting 5 seconds to collect messages"
        time.sleep(5.0)
        
        # listenerpublisher is supposed to repeat our messages back onto /listenerpublisher,
        # make sure we got it
        self.assert_(self.callback_data is not None, "no callback data from listenerpublisher")
        print "Got ", self.callback_data.time
        errorstr = "callback msg field [%%s] from %s does not match"%PUBNODE
        self.assertAlmostEqual(3.14, self.callback_data.time,2,
                               errorstr%"time")
        self.assertEquals(5, len(self.callback_data.joint_states))
        self.assertEquals(6, len(self.callback_data.actuator_states))

        for f in self.callback_data.joint_states:
            self.assertEquals("jointstate", f.name)
            self.assertAlmostEqual(1.0, f.position, 1)
            self.assertAlmostEqual(1.0, f.velocity, 1)   
            self.assertAlmostEqual(1.0, f.applied_effort, 1)
            self.assertAlmostEqual(1.0, f.commanded_effort, 1)               
        for f in self.callback_data.actuator_states:
            self.assertEquals("actuatorstate", f.name)
            self.assertEquals(1.0, f.motor_voltage)
            self.assertEquals(0, f.encoder_count)
            self.assertEquals(0, f.calibration_reading)        
            self.assertEquals(0, f.last_calibration_high_transition)
            self.assertEquals(0, f.num_encoder_errors)
        
if __name__ == '__main__':
    rospy.ready(NAME)
    rostest.run(PKG, NAME, TestMechanismState, sys.argv)
