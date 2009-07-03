import roslib
roslib.load_manifest('tf')
import rostest
import rospy

import tf.msg
import robot_msgs.msg

import _tfX as tf

#help(rospy.Duration())
d = rospy.Duration(3)
print "d.secs", d.secs, d.nsecs
t = tf.Transformer(True, rospy.Duration(10.0))
m = robot_msgs.msg.TransformStamped()
m.header.frame_id = "THISFRAME"
m.parent_id = "PARENT"
m.transform.rotation.w = 1.0
t.setTransform(m)
print "allFramesAsString = \"%s\"" % t.allFramesAsString()
