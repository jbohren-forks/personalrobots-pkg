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
#
# Revision $Id: talker.py 3803 2009-02-11 02:04:39Z rob_wheeler $

import roslib; roslib.load_manifest('pr2_alpha')

import rospy
import unittest
import rostest
import sys
from std_msgs.msg import String

class TestBasicLocalization(unittest.TestCase):

  def setUp(self):
    self.expected_names = sys.argv[1:-1]
    self.received = False
    rospy.Subscriber('success', String, self.callback)
    rospy.init_node('subpub_test', anonymous=True)

  def callback(self, name):
    for i in range(0,len(self.expected_names)):
      if name.data == self.expected_names[i]:
        del self.expected_names[i]
        break
    if len(self.expected_names) == 0:
      self.received = True

  def test_remote_pr2(self):
    while not self.received:
      rospy.sleep(0.1)
    self.assertTrue(self.received)
        
if __name__ == '__main__':
  rostest.run('pr2_alpha', 'full_connection', 
              TestBasicLocalization, sys.argv)


