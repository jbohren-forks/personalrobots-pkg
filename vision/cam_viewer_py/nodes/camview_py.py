import rostools
rostools.updatePath('cam_viewer_py')
from std_msgs.msg import Image
from std_msgs.msg import ImageArray
import rospy
import pyrob.util as ut
import opencv.highgui as hg
import sys
import time

def display_array(iar):
    left  = ut.ros2cv(iar.images[0])
    right = ut.ros2cv(iar.images[1])

    hg.cvShowImage('channel 1', left)
    hg.cvShowImage('channel 2', right)
    hg.cvWaitKey(5)

def display_single(im):
    imcv = ut.ros2cv(im)
    hg.cvShowImage('channel 1', imcv)
    hg.cvWaitKey(5)

hg.cvNamedWindow('channel 1', 1)
hg.cvNamedWindow('channel 2', 1)
rospy.TopicSub('images', ImageArray, display_array)
rospy.TopicSub('image', Image, display_single)
rospy.ready(sys.argv[0])
rospy.spin()













































#bottle = message_bottle()
#class message_bottle:
#    def __init__(self):
#        self.message = None
#
#    def callback(self, iar):
#        l = iar.images[0]
#        r = iar.images[1]
#        #print l.width, l.height, l.compression, l.colorspace, len(l.data), l.data.__class__
#        #print dir(l.data)
#        self.message = iar
#
#    def get_message(self):
#        return self.message
#
#    def empty(self):
#        return self.message == None
#
#
#
#print 'hready'
#while bottle.empty():
#    time.sleep(.033)
#    print 'waiting...'
#    pass


