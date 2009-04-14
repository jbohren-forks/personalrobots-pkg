#!/usr/bin/env python

import roslib
roslib.load_manifest('amcl_player')

import sys
import time
import math

import unittest
import rospy
import rostest
#import tf

from robot_msgs.msg import PoseWithCovariance
from std_srvs.srv import Empty

class TestBasicLocalization(unittest.TestCase):

  def setUp(self):
    self.pose = None

  def pose_cb(self, msg):
    self.pose = msg.pose

  def test_basic_localization(self):
    rospy.wait_for_service('global_localization')
    global_localization = rospy.ServiceProxy('global_localization', Empty)
    resp = global_localization()

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
    rospy.Subscriber('amcl_pose', PoseWithCovariance, self.pose_cb)

    while (rospy.rostime.get_time() - start_time) < target_time:
      time.sleep(0.1)
    self.assertNotEquals(self.pose, None)
    #TODO: compare orientation using pytf
    #tf_pose = tf.pose_stamped_msg_to_bt(self.pose)
    #rotmat = tf_pose.getBasis()
    #print (stderr, rotmat)
    #yaw = 0.0
    #pitch = 0.0
    #roll = 0.0
    #rotmat.getEulerZYX(yaw,pitch,roll)
    #print (stderr, yaw)
    
    self.assertTrue(abs(self.pose.position.x - target_x) <= tolerance_d)
    self.assertTrue(abs(self.pose.position.y - target_y) <= tolerance_d)

if __name__ == '__main__':
  rostest.run('amcl_player', 'amcl_basic_localization', 
              TestBasicLocalization, sys.argv)
