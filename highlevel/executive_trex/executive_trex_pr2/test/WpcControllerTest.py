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

## Waypoint hit test

PKG = 'executive_trex_pr2'

import rostools
rostools.update_path(PKG)

import rospy
from std_msgs.msg import Pose2DFloat32

class WpcControllerTest:
    def __init__(self, targets, tol, start):
        self.targets = targets
        self.tol = tol
        self.start = start
        self.targets_passed = []
        for target in self.targets:
            self.targets_passed.append(False)

    def positionInput2d(self, pos):
        self.targets_passed = self.compareTargets(self.targets_passed, self.targets, self.tol, [pos.x, pos.y])

    def advertiseInitalPose2d(self, name):
        self.pub = rospy.TopicPub("initialpose", Pose2DFloat32)
        
    def publishInitalPose2d(self, name):
        start = Pose2DFloat32()
        start.x = self.start[0]
        start.y = self.start[1]
        start.th = self.start[2]
        #self.pub.publish(start)
    
    def success(self):
        return self.allTargetsFound(self.targets_passed)

    def compareTarget(self, target, tol, pos):
        hit = True
        for i in range(0, len(target)):
            if (target[i] - pos[i] > tol[i] or target[i] - pos[i] < -tol[i]):
                hit = False
        return hit

    def compareTargets(self, targets_passed, targets, tol, pos):
        print "Position: " + str(pos)
        for i in range(0, len(targets)):
            if (self.compareTarget(targets[i], tol, pos)):
                print "Hit target: " + str(i)
                targets_passed[i] = True
        return targets_passed

    def allTargetsFound(self, targets_passed):
        for t in targets_passed:
            if (t == False):
                return False
        return True









