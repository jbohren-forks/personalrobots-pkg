#!/usr/bin/env python

import roslib
roslib.load_manifest('amcl_player')

import sys
import unittest

import rospy
import rostest
import time
import math

from robot_msgs.msg import PoseWithCovariance

class TestBasicLocalization(unittest.TestCase):

  def setUp(self):
    self.pose = None

  def pose_cb(self, msg):
    self.pose = msg.pose

  def test_basic_localization(self):
    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    target_z = float(sys.argv[3])
    target_w = float(sys.argv[4])
    tolerance_d = float(sys.argv[5])
    tolerance_a = float(sys.argv[6])
    target_time = float(sys.argv[7])

    rospy.init_node('test', anonymous=True)
    while(rospy.rostime.get_time() == 0.0):
      time.sleep(0.1)
    start_time = rospy.rostime.get_time()
    rospy.Subscriber('amcl_pose', PoseWithCovariance, self.pose_cb)

    while (rospy.rostime.get_time() - start_time) < target_time:
      time.sleep(0.1)
    self.assertNotEquals(self.pose, None)
    self.assertTrue(abs(self.pose.position.x - target_x) <= tolerance_d)
    self.assertTrue(abs(self.pose.position.y - target_y) <= tolerance_d)
    self.assertTrue(abs(self.pose.orientation.z - target_z) <= tolerance_a)
    self.assertTrue(abs(self.pose.orientation.w - target_w) <= tolerance_a)

if __name__ == '__main__':
  rostest.run('amcl_player', 'amcl_basic_localization', 
              TestBasicLocalization, sys.argv)
