#!/usr/bin/env python

import roslib
roslib.load_manifest('outlet_detection')

import sys
import time
import math

import bullet
import unittest
import rospy
import rostest

from robot_msgs.msg import PoseStamped

class TestBasicLocalization(unittest.TestCase):

  def setUp(self):
    print sys.argv
    self.assertTrue(len(sys.argv) >= 11)

    self.true_position = (float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    self.position_tolerance = float(sys.argv[4])

    self.true_orientation = bullet.Quaternion(float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]), float(sys.argv[8]))
    self.orientation_tolerance = float(sys.argv[9])

    self.time_limit = float(sys.argv[10])

    self.have_pose = False
    self.failed = False

    print 'True position: (%9.6f, %9.6f, %9.6f). Tolerance: %9.6f' %(self.true_position[0], self.true_position[1], self.true_position[2], self.position_tolerance)
    print 'True orientation: (%9.6f, %9.6f, %9.6f, %9.6f). Tolerance: %9.6f' %(self.true_orientation.x(), self.true_orientation.y(), self.true_orientation.z(), self.true_orientation.w(), self.orientation_tolerance)


  def pose_cb(self, msg):
    self.have_pose = True

    dx = self.true_position[0] - msg.pose.position.x
    dy = self.true_position[1] - msg.pose.position.y
    dz = self.true_position[2] - msg.pose.position.z

    dist = math.sqrt(dx*dx + dy*dy + dz*dz)

    print 'New position: (%9.6f, %9.6f, %9.6f). Difference: %9.6f.'%(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z, dist)
      
    if dist > self.position_tolerance:
      self.failed = True

    orientation = bullet.Quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)

    angular_dist = orientation.angle(self.true_orientation)

    print 'New orientation: (%9.6f, %9.6f, %9.6f, %9.6f). Difference: %9.6f' %(orientation.x(), orientation.y(), orientation.z(), orientation.w(), angular_dist)

    if angular_dist > self.orientation_tolerance:
      self.failed = True
    
  def test_detection(self):

    rospy.Subscriber('outlet_detector/outlet_pose', PoseStamped, self.pose_cb)
    rospy.init_node('test', anonymous=True)

    # This test doesn't use actual .bag files, so there's no time feed.
    # We'll use the wall clock to check the time limit.
    start_time = rospy.rostime.get_time()

    while (rospy.rostime.get_time() - start_time) < self.time_limit:
      print 'Waiting for end time %.6f (current: %.6f)'%(self.time_limit,(rospy.rostime.get_time() - start_time))
      time.sleep(0.1)
    self.assertTrue(self.have_pose)
    self.assertFalse(self.failed)

if __name__ == '__main__':
  rostest.run('outlet_detection', 'detection', 
              TestBasicLocalization, sys.argv)

