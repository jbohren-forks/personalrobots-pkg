import roslib
roslib.load_manifest('pytf')

import TransformListener
import time
import rospy
print "Starting"

rospy.init_node("pylistener")
t = TransformListener.TransformListener()

time.sleep(0.5)
for i in xrange(1,100):
    t.get_transform("frame", "parent", rospy.rostime.get_rostime())

print "Done"
