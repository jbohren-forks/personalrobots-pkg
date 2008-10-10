#! /usr/bin/env python

import rostools
rostools.update_path('mechanism_control')
rostools.update_path('robot_mechanism_controllers')


import rospy, sys
from mechanism_control.srv import *
from robot_mechanism_controllers.srv import *
from robot_mechanism_controllers import controllers

def print_usage(exit_code = 0):
    print '''Commands:
    ls                         - List controllers
    set <controller> <command> - Set the controller's commanded value
    setv <controller> <x> <y> <z>  - Set the controller's command as a vector
    get <controller>           - Get the controller's commanded value
    getv <controller>          - Get the controller's vector value 
    profile <controller> <upper turnaround offset> <lower turnaround offset> <upper decel buffer> <lower decel bufer>
                               - Define how far away from joint limit to turn around. Buffers indicate how far from that point to start decelerating. Set to 0 to disable
    sine <controller> <period> <amplitude> <offset>
                               - Define a sine sweep for the Hokuyo laser controller
'''
    sys.exit(exit_code)

if __name__ == '__main__':
    if len(sys.argv) == 1:
        print_usage()
    if sys.argv[1] == 'ls':
        controllers.list_controllers()
    elif sys.argv[1] == 'set':
        if len(sys.argv) != 4:
          print_usage()
        controllers.set_controller(sys.argv[2], float(sys.argv[3]))
    elif sys.argv[1] == 'setv':
        if len(sys.argv) < 6:
            print_usage()
        controllers.set_controller_vector(sys.argv[2], map(float, sys.argv[3:6]))
    elif sys.argv[1] == 'get':
        if len(sys.argv) < 3:
          print_usage()
        controllers.get_controller(sys.argv[2])
    elif sys.argv[1] == 'getv':
        if len(sys.argv) < 3:
            print_usage()
        controllers.get_controller_vector(sys.argv[2])
    elif sys.argv[1] == 'profile':
        controllers.set_profile(sys.argv[2],float(sys.argv[3]),float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]))
    elif sys.argv[1] == 'sine' :
        controllers.set_sine(sys.argv[2],float(sys.argv[3]),float(sys.argv[4]), float(sys.argv[5]))
    elif sys.argv[1] == 'setPosition':
        controllers.set_position(sys.argv[2],float(sys.argv[3]))
    elif sys.argv[1] == 'getPosition':
        controllers.get_position(sys.argv[2])
    else:
        print_usage(1)
