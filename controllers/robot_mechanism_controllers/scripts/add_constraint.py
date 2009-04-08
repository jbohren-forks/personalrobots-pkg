#! /usr/bin/env python

import roslib
roslib.load_manifest('robot_mechanism_controllers')

import rospy, sys
from robot_mechanism_controllers.srv import *
from robot_mechanism_controllers.msg import *


def print_usage(exit_code = 0):
    print '''Commands:
    <name> <args>      - Create filter with name and args
    usage: filter_coeff_client butter 2 .1
    usage: filter_coeff_client butter 2 .1 high

'''
    sys.exit(exit_code)
    
if __name__ == '__main__':
    if len(sys.argv) == 1:
        print_usage()
    else: # Call the service 
      rospy.wait_for_service('/arm_constraint/add_constraints')
      s = rospy.ServiceProxy('/arm_constraint/add_constraints', ChangeConstraints )
      # params:  constraint_id, joint, start force, start nulspace, max force, p, i, d, i_clamp, not_used_param
      resp = s.call(ChangeConstraintsRequest(JointConstraint('upper_arm', 'r_upper_arm_roll_joint', 0, -0.4, 300, 100, 0, 0, 0, 0)))
      print "resp="+ str(resp.add_ok)

