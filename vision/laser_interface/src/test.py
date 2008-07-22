from pkg import *
from std_msgs.msg import Point3DFloat64
from std_msgs.msg import RobotBase2DOdom
import camera as cam
import sys

def debug_me(p):
    print 'received', p.pos.x, p.pos.y, p.pos.th
rospy.TopicSub('odom', RobotBase2DOdom, debug_me)
rospy.ready(sys.argv[0])
rospy.spin()
