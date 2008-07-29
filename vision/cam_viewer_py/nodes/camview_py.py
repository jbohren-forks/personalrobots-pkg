import rostools
rostools.updatePath('cam_viewer_py')
from std_msgs.msg import Image
from std_msgs.msg import ImageArray
import rospy
import sys
import time

class message_bottle:
    def __init__(self):
        self.message = None

    def callback(self, iar):
        l = iar.images[0]
        r = iar.images[1]
        #print l.width, l.height, l.compression, l.colorspace, len(l.data), l.data.__class__
        #print dir(l.data)
        self.message = iar

    def get_message(self):
        return self.message

    def empty(self):
        return self.message == None

bottle = message_bottle()
rospy.TopicSub('images', ImageArray, bottle.callback)
rospy.ready(sys.argv[0])

print 'hready'
while bottle.empty():
    time.sleep(.033)
    print 'waiting...'
    pass


