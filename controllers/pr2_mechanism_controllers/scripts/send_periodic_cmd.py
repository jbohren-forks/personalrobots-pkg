#!/usr/bin/env python

PKG = "pr2_mechanism_controllers"

import rostools; rostools.load_manifest(PKG) 

import sys
import os
import string

import rospy
from std_msgs import *

from pr2_mechanism_controllers.msg import PeriodicCmd
from time import sleep

def print_usage(exit_code = 0):
    print '''Usage:
    send_periodic_cmd.py [controller] [profile] [period] [amplitude] [offset]
       - [profile]   - Possible options are linear, linear_blended, sine
       - [period]    - Time for one entire cycle to execute (in seconds)
       - [amplitude] - Distance max value to min value of profile (In radians for laser_tilt controller)
       - [offset]    - Constant cmd to add to profile (offset=0 results in profile centered around 0)
'''
    sys.exit(exit_code)

if __name__ == '__main__':
    if len(sys.argv) != 6:
        print_usage()

    cmd = PeriodicCmd()
    controller =    sys.argv[1]
    cmd.header =    rostools.msg.Header(None, None, None)
    cmd.profile =   sys.argv[2] 
    cmd.period =    float (sys.argv[3])
    cmd.amplitude = float (sys.argv[4])
    cmd.offset =    float (sys.argv[5])

    print 'Sending Command to %s: ' % controller
    print '  Profile Type: %s' % cmd.profile
    print '  Period:       %f Seconds' % cmd.period
    print '  Amplitude:    %f Radians' % cmd.amplitude
    print '  Offset:       %f Radians' % cmd.offset

    command_publisher = rospy.Publisher(controller + '/set_periodic_cmd', PeriodicCmd)
    rospy.init_node('periodic_cmd_commander', anonymous=True)
    sleep(1)
    command_publisher.publish( cmd )

    sleep(1)

    print 'Command sent!'
