#!/usr/bin/env python

import rostools
rostools.update_path('ethercat_hardware')

import sys, traceback, logging, rospy
from ethercat_hardware.msg import PressureState

NAME = 'pressure_listener'

def callback(data):
    print rospy.get_caller_id(), "I heard %d pressure readings @ %s" % (len(data.data0), data.header.stamp)
    for x in xrange(22):
      print "%04x %04x" % (data.data0[x] & 0xff00, data.data1[x] & 0xff00)
      #print "%04x %04x" % (data.data0[x] & 0xffff, data.data1[x] & 0xffff)
   

def listener_with_user_data():
    rospy.TopicSub("/pressure", PressureState, callback)
    rospy.ready(NAME, anonymous=True)
    rospy.spin()

if __name__ == '__main__':
    try:
        listener_with_user_data()
    except KeyboardInterrupt, e:
        pass
    print "exiting"
