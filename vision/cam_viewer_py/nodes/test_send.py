import rostools
rostools.updatePath('cam_viewer_py')
import rospy
import sys
from std_msgs.msg import Image
import time
import opencv as cv
import opencv.highgui as hg

pub = rospy.TopicPub('image', Image)
rospy.ready(sys.argv[0])

image = cv.cvCreateImage(cv.cvSize(640,480), 8, 1)
cv.cvSet(image, 133)

while not rospy.isShutdown():
    pub.publish(Image(None, 'test', image.width, image.height, 'none', 'mono8', image.imageData))
    time.sleep(.033)

rospy.spin()








