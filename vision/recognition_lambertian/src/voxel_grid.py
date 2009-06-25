


import numpy
import roslib; roslib.load_manifest('recognition_lambertian')
import rospy

from robot_msgs.msg import PointCloud
from recognition_lambertian.srv import *




class VoxelGrid:
    
    def __init__(self, point_coud, x_res, y_res, z_res):
        self.x_res = x_res
        self.y_res = y_res
        self.z_res = z_res
        
    
    
def model_fit(req):
    print "Service called"
    print req.cloud.header
    return ModelFitResponse()
    
    
def setup():
    rospy.init_node('model_fit')
    service = rospy.Service("recognition_lambertian/model_fit", ModelFit, model_fit)
    rospy.spin()
    
def callback(cloud):
    print "I got a point cloud"
        
if __name__ == '__main__':
    setup()
