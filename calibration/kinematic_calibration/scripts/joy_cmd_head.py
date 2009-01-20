#!/usr/bin/env python

import rostools; rostools.update_path('kinematic_calibration') 
import sys
import rospy
from std_msgs.msg import PointStamped, Point
from time import sleep
from joy.msg import Joy 

def joy_callback(data, pub):
    print 'Got joystick Comand'
    if data.buttons[7] == 1:
        pub.publish(PointStamped(rostools.msg.Header(None, None, 'r_wrist_roll_joint'), Point(.17, 0, 0)))
        print 'Sent head Command'

if __name__ == '__main__':
    rospy.init_node('joy_head_commander', anonymous=True)
    head_publisher = rospy.Publisher('head_controller/frame_track_point', PointStamped)


    sub = rospy.Subscriber("/joy", Joy, joy_callback,
	                   head_publisher)

    print 'Ready to send joystick commands as pointhead'
    rospy.spin()
