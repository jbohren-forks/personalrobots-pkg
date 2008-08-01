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
import Image as PImage

def to_cv(image):
    pil_image = PImage.fromstring( "RGB", (image.width, image.height), image.data)
    t = time.time()
    pil_image.save('ros_test.bmp', 'BMP')
    cvim = hg.cvLoadImage('ros_test.bmp')
    print time.time() - t
    return cvim

def print_data(image):
    for i in range(10):
        print image.data[i],
    print ''

def callback_array(iar):
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
    hg.cvWaitKey(5)

def callback_image(im):
    t = time.time()
    cvim = to_cv(im)
    hg.cvShowImage('left', cvim)
    hg.cvWaitKey(5)
    print 'total', time.time() - t

hg.cvNamedWindow('left', 1)
rospy.TopicSub('images', Image, callback_image)
rospy.ready(sys.argv[0])
rospy.spin()




























    #cv_image.imageData = np.frombuffer(image.data, dtype='uint8', count=image.height*image.width*1).tostring()
    #cv_image.imageData = np.array(image.data, dtype='uint8').tostring()
#hg.cvNamedWindow('right', 1)
#rospy.TopicSub('images', ImageArray, callback)
