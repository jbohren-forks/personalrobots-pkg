PKG = 'laser_interface'
import sys, os, subprocess
try:
    rostoolsDir = (subprocess.Popen(['rospack', 'find', 'rostools'], stdout=subprocess.PIPE).communicate()[0] or '').strip()
    sys.path.append(os.path.join(rostoolsDir,'src'))
    import rostools.launcher
    rostools.launcher.updateSysPath(sys.argv[0], PKG, bootstrapVersion="0.6")
except ImportError:
    print >> sys.stderr, "\nERROR: Cannot locate rostools"
    sys.exit(1)  

#from pkg import *
import rospy
from std_msgs.msg import Point3DFloat64
from std_msgs.msg import RobotBase2DOdom
from std_msgs.msg import Pose2DFloat32
import sys
import time

pub = rospy.TopicPub('odom', RobotBase2DOdom)
rospy.ready(sys.argv[0])

while not rospy.isShutdown():
    odom = RobotBase2DOdom(None, Pose2DFloat32(1,2,3), Pose2DFloat32(1,2,4), 1)
    pub.publish(odom)
    time.sleep(.5)
    print 'published'


