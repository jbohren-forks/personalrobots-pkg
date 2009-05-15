#!/usr/bin/env python

import roslib
roslib.load_manifest('2dnav_pr2')

import sys
import time
import math

import unittest
import rospy
import rostest

from robot_msgs.msg import PoseStamped
from nav_robot_actions.msg import MoveBaseState

class TestGoto(unittest.TestCase):

  def setUp(self):
    self.success = False
    self.pub = rospy.Publisher('move_base_local/activate', PoseStamped)
    self.sub = rospy.Subscriber('move_base_local/feedback', MoveBaseState, self.cb)
    rospy.init_node('test', anonymous=True)

  def cb(self, msg):
    dx = abs(msg.feedback.pose.position.x - self.target_x)
    dy = abs(msg.feedback.pose.position.y - self.target_y)
    print 'dx: %.3f dy: %.3f (%.3f)'%(dx,dy,self.tolerance_d)
    # TODO: compare orientation
    if dx < self.tolerance_d and dy < self.tolerance_d:
      self.success = True

  def test_basic_localization(self):
    self.target_x = float(sys.argv[1])
    self.target_y = float(sys.argv[2])
    self.target_a = float(sys.argv[3])
    self.tolerance_d = float(sys.argv[4])
    self.tolerance_a = float(sys.argv[5])
    target_time = float(sys.argv[6])

    goal = PoseStamped()
    goal.header.frame_id = 'odom_combined'
    goal.pose.position.x = self.target_x
    goal.pose.position.y = self.target_y
    goal.pose.position.z = 0.0
    # TODO
    goal.pose.orientation.x = 0.0
    goal.pose.orientation.y = 0.0
    goal.pose.orientation.z = 0.0
    goal.pose.orientation.w = 1.0

    while(rospy.rostime.get_time() == 0.0):
      print 'Waiting for initial time publication'
      time.sleep(0.1)

    self.pub.publish(goal)
    start_time = rospy.rostime.get_time()

    while not self.success and (rospy.rostime.get_time() - start_time) < target_time:
      print 'Waiting for end time %.6f (current: %.6f)'%(target_time,(rospy.rostime.get_time() - start_time))
      time.sleep(0.1)
    print 'Waited for end time %.6f (current: %.6f)'%(target_time,(rospy.rostime.get_time() - start_time))
    self.assertTrue(self.success)

if __name__ == '__main__':
  rostest.run('2dnav_pr2', 'move_base_local', TestGoto, sys.argv)

