import rostools
rostools.updatePath('cam_viewer_py')
from std_msgs.msg import Image
from std_msgs.msg import ImageArray
import rospy
import sys
import time
import numpy as np
import opencv as cv
import opencv.highgui as hg

def to_cv(image, cv_image=None):
    if cv_image == None:
        cv_image = cv.cvCreateImage(cv.cvSize(image.width, image.height), 8, 1)

    
    cv_image.imageData = np.array(image.data, dtype='uint8').tostring()
    return cv_image

def print_data(image):
    for i in range(10):
        print image.data[i],

def callback(iar):
    left = iar.images[0]
    right = iar.images[1]

    start   = time.time()
    leftcv  = to_cv(left)
    rightcv = to_cv(right)
    print '%.4f' % (time.time() - start)

    print iar.header.seq,
    print_data(left)
    hg.cvShowImage('left', leftcv)
    hg.cvShowImage('right', rightcv)
    hg.cvWaitKey(10)

hg.cvNamedWindow('left', 1)
hg.cvNamedWindow('right', 1)
rospy.TopicSub('images', ImageArray, callback)
rospy.ready(sys.argv[0])
rospy.spin()
