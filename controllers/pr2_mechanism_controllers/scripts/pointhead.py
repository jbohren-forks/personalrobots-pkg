#!/usr/bin/env python

PKG = "pr2_mechanism_controllers"

import rostools; rostools.update_path(PKG) 

import sys
import os
import string

import rospy

from pr2_mechanism_controllers.srv import *

def point_head_client(pan, tilt):

    # NOTE: you don't have to call rospy.init_node() to make calls against
    # a service. This is because service clients do not have to be
    # nodes.

    # block until the add_two_ints service is available
    print "waiting for service"
    rospy.wait_for_service('head_controller/set_command_array')
    
    try:
        # create a handle to the add_two_ints service
        send_cmd= rospy.ServiceProxy('head_controller/set_command_array', SetJointCmd)
        
        print "Requesting %f+%f"%(pan, tilt)
        
        # simplified style
        resp2 = send_cmd.call(SetJointCmdRequest([pan, tilt],[0.0, 0.0],[0.0, 0.0],['head_pan_joint', 'head_tilt_joint']))
        print resp2.positions, resp2.names
        return resp2
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [pan tilt]"%sys.argv[0]

if __name__ == "__main__":
    
    if len(sys.argv) != 3:
        print usage()
        sys.exit(1)
    else:
        point_head_client(float(sys.argv[1]), float(sys.argv[2]))
