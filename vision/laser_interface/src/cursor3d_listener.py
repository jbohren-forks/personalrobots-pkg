from pkg import *
from std_msgs.msg import Point3DFloat64
import sys

class Echo:
    def __init__(self):
        self.count = 0

    def echo(self, point):
        if self.count % 10 == 0:
            print 'x y z', point.x, point.y, point.z, self.count
        self.count = self.count + 1

rospy.TopicSub(CURSOR_TOPIC, Point3DFloat64, Echo().echo)
rospy.ready(sys.argv[0])
rospy.spin()





