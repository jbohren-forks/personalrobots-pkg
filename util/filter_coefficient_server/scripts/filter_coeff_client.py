#! /usr/bin/env python

import rostools
rostools.update_path('filter_coefficient_server')

import rospy, sys
from filter_coefficient_server.srv import *


def print_usage(exit_code = 0):
    print '''Commands:
    <name> <args>      - Create controller with name and args
    usage: filter_coeff_client butter 2 .1
    usage: filter_coeff_client butter 2 .1 high

'''
    sys.exit(exit_code)
    
if __name__ == '__main__':
    if len(sys.argv) == 1:
        print_usage()
    else: # Call the service 
      rospy.wait_for_service('filter_coeffs')
      s = rospy.ServiceProxy('filter_coeffs', Filter)
      resp = s.call(FilterRequest(sys.argv[1],sys.argv[2:]))
      print "a="+ str(resp.a)
      print "b="+ str(resp.b)
