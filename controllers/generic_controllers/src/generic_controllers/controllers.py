#! /usr/bin/env python

import rostools
rostools.update_path('mechanism_control')
rostools.update_path('generic_controllers')


import rospy, sys
from mechanism_control.srv import *
from generic_controllers.srv import *

def print_usage(exit_code = 0):
    print '''Commands:
    ls                         - List controllers
    set <controller> <command> - Set the controller's commanded value
    get <controller>           - Get the controller's commanded value
    profile <controller> <mode> <period> <amplitude> <offset> - Set profile 3 for sawtooth 4 for sinewave'''
    sys.exit(exit_code)

def list_controllers():
    s = rospy.ServiceProxy('list_controllers', ListControllers)
    resp = s.call(ListControllersRequest())
    for t in resp.controllers:
        print t

def set_controller(controller, command):
    s = rospy.ServiceProxy(controller + '/set_command', SetCommand)
    resp = s.call(SetCommandRequest(command))
    print resp.command

def get_controller(controller):
    s = rospy.ServiceProxy(controller + '/get_actual', GetActual)
    resp = s.call(GetActualRequest())
    print str(resp.time) + ": " + str(resp.command)

def set_profile(controller, profile,period,amplitude,offset):
    s = rospy.ServiceProxy(controller + '/set_profile', SetProfile)
    resp = s.call(SetProfileRequest(profile,period,amplitude,offset))
    print str(resp.time)


if __name__ == '__main__':
    if len(sys.argv) == 1:
        print_usage()
    if sys.argv[1] == 'ls':
        list_controllers()
    elif sys.argv[1] == 'set':
        if len(sys.argv) != 4:
          print_usage()
        set_controller(sys.argv[2], float(sys.argv[3]))
    elif sys.argv[1] == 'get':
        if len(sys.argv) != 3:
          print_usage()
        get_controller(sys.argv[2])
    elif sys.argv[1] == 'profile':
        set_profile(sys.argv[2],int(sys.argv[3]),float(sys.argv[4]),float(sys.argv[5]),float(sys.argv[6]))
    else:
        print_usage(1)
