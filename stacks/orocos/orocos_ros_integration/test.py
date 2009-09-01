#!/usr/bin/env python
import roslib; roslib.load_manifest('rospy_tutorials')
import rospy
from std_msgs.msg import *

def callback(data):
    print data.data

pub = rospy.Publisher('/RTT/Deployer/Comp1/ReadPort1',Float64)
n=rospy.init_node('test',anonymous=True)
fl=Float64(0.0)

while( not rospy.is_shutdown()):
    fl.data+=1
    pub.publish(fl)
    rospy.sleep(0.01)
    
