#!/usr/bin/env python

import rostools; rostools.update_path('kinematic_calibration') 
import sys
import rospy
from roscpp.msg import Empty
from time import sleep
from joy.msg import Joy 

def joy_callback(data, pub):
	print 'Got joystick Comand'
	if data.buttons[5] == 1:
		cmd = Empty() ;
		pub.publish(cmd)
		print 'Sent Empty Command'


if __name__ == '__main__':
	topic = 'cmd'
	command_publisher = rospy.Publisher(topic, Empty)
    	rospy.init_node('goto_next', anonymous=True)
	sub = rospy.Subscriber("/joy", Joy, joy_callback,
			       command_publisher)

	print 'Ready to send joystick commands as cmd'
	rospy.spin()
