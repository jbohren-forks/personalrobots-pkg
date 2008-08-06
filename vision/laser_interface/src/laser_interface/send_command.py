from pkg import *
from std_msgs.msg import Point3DFloat64
import sys, time
import opencv as cv
import opencv.highgui as hg
import camera as cam
import util as ut
import random_forest as rf
import dimreduce as dr
from laser_detector import *
from threading import RLock

topic = rospy.TopicPub(CURSOR_TOPIC, Point3DFloat64)
rospy.ready(sys.argv[0])

if not rospy.isShutdown():
    time.sleep(1)
    topic.publish(Point3DFloat64(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])))
    print 'sent message'

