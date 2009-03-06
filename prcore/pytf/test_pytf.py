#!/usr/bin/env python
import roslib
roslib.load_manifest('pytf')

import TransformListener
import time
import rospy
print "Starting"

rospy.init_node("pylistener")
t = TransformListener.TransformListener()

time.sleep(0.5)
for i in xrange(1,10):
    print "Looping"
    current_time = rospy.rostime.get_rostime()
    if not t.can_transform("frame", "parent", current_time):
        print "waiting to try again"
        time.sleep(0.1)
    try:
        t.get_transform("frame", "parent", current_time)
        print "Successfully transformed"
    except ValueError:
        print "valueerror"

print "Done"
