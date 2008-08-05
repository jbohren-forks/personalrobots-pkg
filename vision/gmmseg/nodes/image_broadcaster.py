'''
@package image_broadcaster
Broadcast image given on command line to topic 'image'.
'''
import rostools
rostools.updatePath('gmmseg')
import rospy
import std_msgs.msg.Image as RImage 
import opencv.highgui as hg
import opencv as cv
import sys
import time

rospy.ready(sys.argv[0])
pub_channel = rospy.TopicPub('image', RImage)
cv_image    = hg.cvLoadImage(sys.argv[1])
cv.cvCvtColor(cv_image, cv_image, cv.CV_BGR2RGB )

count = 1
while not rospy.isShutdown():
    image = RImage(None, 'static', cv_image.width, cv_image.height, 
            '', 'rgb24', cv_image.imageData)
    pub_channel.publish(image)
    print 'published', count
    count = count + 1
    time.sleep(.3)
