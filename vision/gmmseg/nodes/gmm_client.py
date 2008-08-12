"""
@package gmm_client
Testing node that calls the service gmm_segment with parameters
given on command line.
"""

import rostools
rostools.updatePath('gmmseg')
import gmmseg.srv as srv
import rospy
import sys

rospy.wait_for_service('hrl_grasp')
segment_proxy = rospy.ServiceProxy('hrl_grasp', srv.hrl_grasp)
result = segment_proxy.call(srv.hrl_graspRequest(float(sys.argv[1])))
print result.x, result.y, result.z, result.theta

