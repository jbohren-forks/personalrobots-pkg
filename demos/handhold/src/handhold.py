#!/usr/bin/python

import roslib; roslib.load_manifest('handhold')
import rospy
from tf import TransformListener
from robot_msgs.msg import PoseStamped
from robot_msgs.msg import PoseDot
#import numpy

rospy.init_node("handhold")

t = TransformListener()
rate = rospy.Rate(100)

q_target = None

base_pub = rospy.Publisher('/cmd_vel', PoseDot);

cmd_vel = PoseDot()

error_x = 0
error_y = 0
error_rotation = 0

while not rospy.is_shutdown():
  gripper_pose = PoseStamped()
  gripper_pose.header.frame_id = '/r_gripper_tool_frame'
  if(t.canTransform('/base_link', '/r_gripper_tool_frame', gripper_pose.header.stamp)):  
    q = t.transformPose('/base_link', gripper_pose)
    if not q_target:
      q_target = q
    error_x = q_target.pose.position.x - q.pose.position.x
    error_y = q_target.pose.position.y - q.pose.position.y
    error_rotation = q_target.pose.orientation.z - q.pose.orientation.z
    print (error_rotation)

  cmd_vel.vel.vx = -10.0 * error_x
  cmd_vel.ang_vel.vz = -10.0 * error_y + error_rotation * -5.0
  cmd_vel.vel.vy = error_rotation * 2.0
  base_pub.publish(cmd_vel)

  rate.sleep();

#Command gripper to target pose

#while not node.is_shutdown():
  #Get pose of gripper
  #Compare to target pose of gripper
  #Transform to base
  #Publish to cmd_vel
