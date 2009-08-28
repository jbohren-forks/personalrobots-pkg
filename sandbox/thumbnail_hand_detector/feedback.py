#!/usr/bin/env python
import roslib; roslib.load_manifest('thumbnail_hand_detector')
import rospy
from std_msgs.msg import String
import os
import sys
import time

def callback(data):
    os.system("clear");
    os.system("cat hand.txt")
    time.sleep(.1);
    os.system("clear");
    
def listener():
    os.system("clear");
    rospy.init_node('handy')
    rospy.Subscriber("/headcart/hands", String, callback)
    rospy.spin()
        
if __name__ == '__main__':
    listener()
