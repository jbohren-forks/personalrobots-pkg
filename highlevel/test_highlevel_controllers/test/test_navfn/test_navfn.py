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
PKG = 'test_highlevel_controllers'
NAME = 'test_navfn'

import roslib
roslib.load_manifest(PKG)
roslib.load_manifest('rostest')

import unittest, sys, os, math
import rospy, rostest, time
from  robot_actions.msg import *

class TestMoveBaseNavfn(unittest.TestCase):
    def __init__(self, *args):
        super(TestMoveBaseNavfn, self).__init__(*args)
        self.success = False
        self.fail = False
        self.goal = Pose2D()
        # Do initialization that rospy demands
        self.goal.boundary.points = []
        self.goal.boundary.color.r = 0.0
        self.goal.boundary.color.g = 0.0
        self.goal.boundary.color.b = 0.0
        self.goal.boundary.color.a = 0.0

        self.goal.header.frame_id = "/map"
        self.goal.x = float(sys.argv[1]) #19
        self.goal.y = float(sys.argv[2]) #29
        self.goal.th = float(sys.argv[3]) #1
        self.node_started = False
        
    def diff(self, a, b, rt = 1, tt = 0.1):
        #print a.x, b.x
        dx = a.x  - b.x
        dy = a.y  - b.y
        dt = a.th - b.th

        pi = 3.141592654
        pi2 = pi * 2.0
        while (dt > pi):
            dt = dt - pi2
        while (dt < -pi):
            dt = dt + pi2
            
        #print dx, dy, dt
        dr = dx * dx + dy * dy
        return (dr < rt and dt * dt < tt)

        

    def stateInput(self, state):
        self.node_started = True
        if (self.diff(state.feedback, state.goal) and self.diff(self.goal, state.goal, 0.0001, 0.0001)):
            self.success = True
    
    def test_navfn(self):
        rospy.init_node(NAME, anonymous=True)
        sub = rospy.Subscriber("/move_base_node/feedback", MoveBaseState, self.stateInput)
        while(not self.node_started):
            print "Waiting for move_base..."
            time.sleep(1.0)
        time.sleep(2.0)
        self.goal_pub = rospy.Publisher("/goal", Pose2D)
        time.sleep(2.0)
        print "Publishing goal...", self.goal.x
        self.goal_pub.publish(self.goal)
        print "Running..."
        while not rospy.is_shutdown() and not self.success and not self.fail:
            time.sleep(0.1)
        print "Done"
        if (not self.node_started):
            print "Non-starter."
            raise("Node never started")
        print "Assert:"
        self.assert_(self.success)
        
    


if __name__ == '__main__':
    rostest.run(PKG, sys.argv[0], TestMoveBaseNavfn, sys.argv) #, text_mode=True)


