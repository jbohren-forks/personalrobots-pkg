#!/usr/bin/env python

import rostools
rostools.update_path('2dnav_pr2')

import sys, traceback, logging, rospy
from std_msgs.msg import RobotBase2DOdom

NAME = 'pose_listener'

def odom_callback(data):
    print "odom: %d.%d %.2f, %.2f, %.2f" % (data.header.stamp.secs, data.header.stamp.nsecs, data.pos.x, data.pos.y, data.pos.th)

def localized_callback(data):
    print "localized: %d.%d %.2f, %.2f, %.2f" % (data.header.stamp.secs, data.header.stamp.nsecs, data.pos.x, data.pos.y, data.pos.th)

def listener_with_user_data():
    rospy.TopicSub("/odom", RobotBase2DOdom, odom_callback)
    rospy.TopicSub("/localizedpose", RobotBase2DOdom, localized_callback)
    rospy.ready(NAME, anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener_with_user_data()
    except KeyboardInterrupt, e:
        pass
    print "exiting"
