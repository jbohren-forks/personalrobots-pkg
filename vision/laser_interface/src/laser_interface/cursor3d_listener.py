from pkg import *
from std_msgs.msg import Point3DFloat64
import sys

class Echo:
    def echo(self, point):
        print 'x y z', point.x, point.y, point.z

rospy.TopicSub(CURSOR_TOPIC, Point3DFloat64, Echo().echo)
rospy.ready(sys.argv[0])
rospy.spin()





