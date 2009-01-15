#!/usr/bin/env python

import rostools
rostools.update_path('trajectory_rollout')

import sys, traceback, logging, rospy
from robot_msgs.msg import Planner2DGoal

NAME = 'goal_listener'

def callback(data):
    print "Goal: %d, %d, %d" % (data.goal.x, data.goal.y, data.goal.th)

def listener_with_user_data():
    rospy.TopicSub("/goal", Planner2DGoal, callback)
    rospy.ready(NAME, anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener_with_user_data()
    except KeyboardInterrupt, e:
        pass
    print "exiting"
