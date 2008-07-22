from pkg import *
from std_msgs.msg import Point3DFloat64
from std_msgs.msg import RobotBase2DOdom
from std_msgs.msg import Pose2DFloat32
import camera as cam
import sys
import time

pub = rospy.TopicPub('odom', RobotBase2DOdom)
rospy.ready(sys.argv[0])

while True:
    odom = RobotBase2DOdom('header', Pose2DFloat32(1,2,3), Pose2DFloat32(1,2,4), 1)
    time.sleep(.5)
    print 'published'


