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

from pkg import *
from std_msgs.msg import Point3DFloat64
from std_msgs.msg import RobotBase2DOdom
import sys

def debug_me(p):
    print 'received', p.pos.x, p.pos.y, p.pos.th
rospy.TopicSub('odom', RobotBase2DOdom, debug_me)
rospy.ready('test')
rospy.spin()
