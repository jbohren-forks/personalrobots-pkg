#! /usr/bin/env python

import roslib
roslib.load_manifest('iir_filters')

import rospy, sys
from iir_filters.srv import *


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
      rospy.wait_for_service('filter_coeffs')
      s = rospy.ServiceProxy('filter_coeffs', Filter)
      resp = s.call(FilterRequest(sys.argv[1],sys.argv[2:]))
      print "a="+ str(resp.a)
      print "b="+ str(resp.b)
