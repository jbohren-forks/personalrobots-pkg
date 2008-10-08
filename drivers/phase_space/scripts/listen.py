#!/usr/bin/env python

import rostools
rostools.update_path('phase_space')

import sys, traceback, logging, rospy
from phase_space.msg import PhaseSpaceSnapshot
from phase_space.msg import PhaseSpaceMarker

NAME = 'phase_space_listener'

def callback(data):
    timeSeconds = data.header.stamp.secs + data.header.stamp.nsecs*1e-9
    print rospy.get_caller_id(), "I heard [%.6f]"%(timeSeconds)
    print "  Frame #:     %u" % (data.frameNum)
    print "  Num Markers: %u" % (len(data.markers))
    for j in data.markers:
      #print "  ID: %d" % j.id
      #print "    x:         %.4f" % j.location.x
      #print "    y:         %.4f" % j.location.y
      #print "    z:         %.4f" % j.location.z
      #print "    cond:      %.4f" % j.condition
      print "  ID=%03d Pos=(% 08.4f,% 08.4f,% 08.4f) Cond=%f" % (j.id, j.location.x, j.location.y, j.location.z, j.condition)
    print " "

def listener_with_user_data():
    print "Starting Listener"
    rospy.TopicSub("/phase_space_snapshot", PhaseSpaceSnapshot, callback)
    rospy.ready(NAME, anonymous=True)
    print "Spinning"
    rospy.spin()

if __name__ == '__main__':
    try:
        listener_with_user_data()
    except KeyboardInterrupt, e:
        pass
    print "exiting"
