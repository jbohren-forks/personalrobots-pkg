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
import time

from robot_msgs.msg import PoseStamped
from robot_actions.msg import ActionStatus
from nav_robot_actions.msg import MoveBaseState
from tf.msg import tfMessage

# Are these defined somewhere else?
def normalize(z):
  return math.atan2(math.sin(z),math.cos(z))
def angle_diff(a, b):
  a = normalize(a)
  b = normalize(b)
  d1 = a-b
  d2 = 2*math.pi - abs(d1)
  if d1 > 0:
    d2 *= -1.0
  if abs(d1) < abs(d2):
    return d1
  else:
    return d2

class TestGoto(unittest.TestCase):

  def setUp(self):
    self.current_pose = None
    self.pub = rospy.Publisher('move_base_local/activate', PoseStamped)
    self.sub = rospy.Subscriber('move_base_local/feedback', MoveBaseState, self.feedback_cb)
    self.sub = rospy.Subscriber('tf_message', tfMessage, self.tf_cb)

    self.active = False
    self.ontarget = False
    self.action_succeeded = False

    self.have_goal = False

    rospy.init_node('test', anonymous=True)
    # HACK: give move_base_local time to connect
    time.sleep(2)

  def tf_cb(self, msg):
    if self.active:
      for t in msg.transforms:
        if t.parent_id == 'odom_combined':
          tx = t.transform
  
          # Convert Transform.msg to bullet
          current_pose = bullet.Transform(bullet.Quaternion(tx.rotation.x,
                                                            tx.rotation.y,
                                                            tx.rotation.z,
                                                            tx.rotation.w),
                                          bullet.Vector3(tx.translation.x,
                                                         tx.translation.y,
                                                         tx.translation.z))

          print 'pose: %.3f %.3f %.3f'%(current_pose.getOrigin().x(),
                                       current_pose.getOrigin().y(),
                                       current_pose.getBasis().getEulerZYXYaw())
          if not self.have_goal:
            self.goal = current_pose * self.offset
            self.have_goal = True
            print 'offs: %.3f %.3f'%(self.offset.getOrigin().x(),
                                     self.offset.getOrigin().y())
            print 'goal: %.3f %.3f %.3f'%(self.goal.getOrigin().x(),
                                          self.goal.getOrigin().y(),
                                          self.goal.getBasis().getEulerZYXYaw())
    
          # Did the robot go where we told it?
          dx = abs(current_pose.getOrigin().x() - self.goal.getOrigin().x())
          dy = abs(current_pose.getOrigin().y() - self.goal.getOrigin().y())
          da = abs(angle_diff(current_pose.getBasis().getEulerZYXYaw(),
                              self.goal.getBasis().getEulerZYXYaw()))
          print 'dx: %.3f dy: %.3f (%.3f) da: %.3f (%.3f)'%(dx,dy,self.tolerance_d,da,self.tolerance_a)
          # TODO: compare orientation
          if dx < self.tolerance_d and dy < self.tolerance_d and da < self.tolerance_a:
            self.ontarget = True
            print 'On target'
          else:
            self.ontarget = False
            print 'Off target'

  def feedback_cb(self, msg):
    # Has our new goal been taken up?
    if not self.active and msg.status.value == ActionStatus.ACTIVE:
      self.active = True
      print 'Action is now active'

    if self.active:
      # Did the action report success?
      if not self.action_succeeded and msg.status.value == ActionStatus.SUCCESS:
        self.action_succeeded = True
        print 'Action succeeded'

  def test_basic_localization(self):
    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])
    target_a = float(sys.argv[3])
    self.offset = bullet.Transform(bullet.Quaternion(target_a, 0, 0),
                                   bullet.Vector3(target_x, target_y, 0))

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
    goal.pose.position.x = target_x
    goal.pose.position.y = target_y
    goal.pose.position.z = 0
    # Use bullet and tf to build a quaternion from the user-specified yaw
    goal.pose.orientation = tf.quaternion_bt_to_msg(bullet.Quaternion(target_a,0,0))

    self.pub.publish(goal)
    start_time = rospy.rostime.get_time()

    while (not self.action_succeeded or not self.ontarget) and (rospy.rostime.get_time() - start_time) < target_time:
      print 'Waiting for end time %.6f (current: %.6f)'%(target_time,(rospy.rostime.get_time() - start_time))
      time.sleep(0.1)
    print 'Waited for end time %.6f (current: %.6f)'%(target_time,(rospy.rostime.get_time() - start_time))
    self.assertTrue(self.action_succeeded)
    self.assertTrue(self.ontarget)

if __name__ == '__main__':
  rostest.run('2dnav_pr2', 'move_base_local', TestGoto, sys.argv)

