#!/usr/bin/env python
import roslib; roslib.load_manifest('outlet_detection')

import rospy
from robot_msgs.msg import *

def callback(data):
    pos = data.pose.position
    ori = data.pose.orientation
    print "%f %f %f %f %f %f %f" % (pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w)

def main():
    rospy.init_node('write_poses')
    rospy.Subscriber('/outlet_detector/outlet_pose', PoseStamped, callback)
    #rospy.Subscriber('/plug_detector/plug_pose', PoseStamped, callback)
    rospy.spin()

if __name__ == '__main__': main()
