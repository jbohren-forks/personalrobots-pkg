#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
#  * Neither the name of Willow Garage, Inc. nor the names of its
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

PKG = 'object_names'
import roslib; roslib.load_manifest(PKG)

import rospy

import unittest

from object_names.srv import *

class TestContent(unittest.TestCase):


    def testBasicNames(self):
        rospy.wait_for_service('name_to_color')

        name_2_color = rospy.ServiceProxy('name_to_color', Name2Color)

        error_c=name_2_color("error");
        self.assertAlmostEqual(error_c.color.r,  1.0,2);
        self.assertAlmostEqual(error_c.color.g,  0.0,2);
        self.assertAlmostEqual(error_c.color.b,  0.0,2);

        wall_c=name_2_color("wall");
        self.assertAlmostEqual(wall_c.color.r,   0.5,2);
        self.assertAlmostEqual(wall_c.color.g,   0.5,2);
        self.assertAlmostEqual(wall_c.color.b,   0.0,2);


from threading import Thread

if __name__ == "__main__":
    import rostest
    rospy.init_node("test_content");

    rostest.rosrun('object_names','test_colors',TestContent)

