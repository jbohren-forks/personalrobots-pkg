from pkg import *
from std_msgs.msg import Position
import sys

class Echo:
    def echo(self, point):
        print 'x y z', point.x, point.y, point.z

rospy.TopicSub(CURSOR_TOPIC, Position, Echo().echo)
rospy.ready(sys.argv[0])
rospy.spin()





