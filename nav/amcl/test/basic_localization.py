#!/usr/bin/env python

import roslib
roslib.load_manifest('amcl')

import sys
import time
import math

import unittest
import rospy
import rostest
#import tf

from robot_msgs.msg import PoseWithCovariance
from tf.msg import tfMessage

class TestBasicLocalization(unittest.TestCase):

  def setUp(self):
    self.tf = None

  def tf_cb(self, msg):
    for t in msg.transforms:
      if t.parent_id == 'map':
        self.tf = t.transform

  def test_basic_localization(self):
    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    target_a = float(sys.argv[3])
    tolerance_d = float(sys.argv[4])
    tolerance_a = float(sys.argv[5])
    target_time = float(sys.argv[6])

    rospy.init_node('test', anonymous=True)
    while(rospy.rostime.get_time() == 0.0):
      time.sleep(0.1)
    start_time = rospy.rostime.get_time()
    # This should be replace by a pytf listener
    rospy.Subscriber('tf_message', tfMessage, self.tf_cb)

    while (rospy.rostime.get_time() - start_time) < target_time:
      time.sleep(0.1)
    self.assertNotEquals(self.tf, None)
    self.assertTrue(abs(self.tf.translation.x - target_x) <= tolerance_d)
    self.assertTrue(abs(self.tf.translation.y - target_y) <= tolerance_d)
    #TODO: check orientation

if __name__ == '__main__':
  rostest.run('amcl', 'amcl_basic_localization', 
              TestBasicLocalization, sys.argv)
