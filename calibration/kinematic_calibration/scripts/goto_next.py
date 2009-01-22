#!/usr/bin/env python

import rostools; rostools.update_path('kinematic_calibration') 
import sys
import rospy
from roscpp.msg import Empty
from time import sleep

if __name__ == '__main__':
	prefix = 'arm_commander'
	topic = prefix + '/goto_next'
	command_publisher = rospy.Publisher(topic, Empty)
    	rospy.init_node('goto_next', anonymous=True)
	sleep(1)
	cmd = Empty()
	command_publisher.publish( cmd )
    	sleep(1)
	print 'Command sent!'
