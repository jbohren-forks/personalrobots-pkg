#!/usr/bin/env python

import roslib; roslib.load_manifest('2dnav_pr2')

import sys, traceback, logging, rospy
from deprecated_msgs.msg import RobotBase2DOdom

NAME = 'pose_listener'

def odom_callback(data):
    print "odom: %d.%d %.2f, %.2f, %.2f" % (data.header.stamp.secs, data.header.stamp.nsecs, data.pos.x, data.pos.y, data.pos.th)

def localized_callback(data):
    print "localized: %d.%d %.2f, %.2f, %.2f" % (data.header.stamp.secs, data.header.stamp.nsecs, data.pos.x, data.pos.y, data.pos.th)

def pose_listen():
    rospy.Subscriber("/odom", RobotBase2DOdom, odom_callback)
    rospy.Subscriber("/localizedpose", RobotBase2DOdom, localized_callback)
    rospy.init_node(NAME, anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        pose_listen()
    except KeyboardInterrupt, e:
        pass
    print "exiting"
