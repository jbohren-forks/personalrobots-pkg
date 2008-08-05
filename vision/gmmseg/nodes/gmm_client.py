import rostools
rostools.updatePath('gmmseg')
import gmmseg.srv as srv
import rospy

rospy.wait_for_service('gmm_segment')
segment_proxy = rospy.ServiceProxy('gmm_segment', srv.hrl_grasp)
result = segment_proxy.call(srv.hrl_graspRequest(10))
print result.x, result.y, result.z, result.theta

