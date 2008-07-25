#PKG = 'laser_interface'
#import sys, os, subprocess
#try:
#    rostoolsDir = (subprocess.Popen(['rospack', 'find', 'rostools'], stdout=subprocess.PIPE).communicate()[0] or '').strip()
#    sys.path.append(os.path.join(rostoolsDir,'src'))
#    import rostools.launcher
#    rostools.launcher.updateSysPath(sys.argv[0], PKG, bootstrapVersion="0.6")
#except ImportError:
#    print >> sys.stderr, "\nERROR: Cannot locate rostools"
#    sys.exit(1)  
#
##from pkg import *
#import rospy
#from std_msgs.msg import Point3DFloat64
#from std_msgs.msg import RobotBase2DOdom
#import sys
#
#def debug_me(p):
#    print 'received', p.pos.x, p.pos.y, p.pos.th
#rospy.TopicSub('odom', RobotBase2DOdom, debug_me)
#rospy.ready('test')
#rospy.spin()

import opencv as cv
import opencv.highgui as hg
import util as ut

def swap_br(npimage):
    b = npimage[:,:,0].copy()
    r = npimage[:,:,2].copy()
    npimage[:,:,0] = r
    npimage[:,:,2] = b
    return npimage

image   = hg.cvLoadImage('1frame489.png')
npimg   = ut.cv2np(image)
swapped = swap_br(npimg.copy())
cvimg   = ut.np2cv(swapped)

hg.cvNamedWindow('hai', 1)
hg.cvShowImage('hai', cvimg)
hg.cvWaitKey(10)























