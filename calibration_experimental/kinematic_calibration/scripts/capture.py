#!/usr/bin/env python

import roslib; roslib.load_manifest('kinematic_calibration') 
import sys
import rospy
from kinematic_calibration.msg import Capture
from time import sleep

if __name__ == '__main__':
	command_publisher = rospy.Publisher("capture", Capture)
    	rospy.init_node('capture_node', anonymous=True)
	sleep(1)
	cmd = Capture()
	command_publisher.publish( cmd )
    	sleep(1)
	print 'Command sent!'
