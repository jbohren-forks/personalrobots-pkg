#!/usr/bin/env python

import roslib
roslib.load_manifest('2dnav_pr2')

import sys
import time
import math

import unittest
import rospy
import rostest
import tf
import bullet

from robot_msgs.msg import PoseStamped
from nav_robot_actions.msg import MoveBaseState

class TestGoto(unittest.TestCase):

  def setUp(self):
    self.success = False
    self.current_pose = None
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

    while rospy.rostime.get_time() == 0.0:
      print 'Waiting for initial time publication'
      time.sleep(0.1)

    # Construct goal a robot-centric frame; let move_base_local do the work
    goal = PoseStamped()
    goal.header.stamp = rospy.get_rostime()
    goal.header.frame_id = 'base_link'
    goal.pose.position.x = self.target_x
    goal.pose.position.y = self.target_y
    goal.pose.position.z = 0
    # Use bullet and tf to build a quaternion from the user-specified yaw
    goal.pose.orientation = tf.quaternion_bt_to_msg(bullet.Quaternion(self.target_a,0,0))

    self.pub.publish(goal)
    start_time = rospy.rostime.get_time()

    while not self.success and (rospy.rostime.get_time() - start_time) < target_time:
      print 'Waiting for end time %.6f (current: %.6f)'%(target_time,(rospy.rostime.get_time() - start_time))
      time.sleep(0.1)
    print 'Waited for end time %.6f (current: %.6f)'%(target_time,(rospy.rostime.get_time() - start_time))
    self.assertTrue(self.success)

if __name__ == '__main__':
  rostest.run('2dnav_pr2', 'move_base_local', TestGoto, sys.argv)

