#!/usr/bin/env python
import roslib; roslib.load_manifest('outlet_detection')

import rospy
from robot_msgs.msg import *
import threading
import numpy
from math import acos, degrees

base_ori = None
angles = []

def pose_cb(data):
    global base_ori, angles
    
    ori = data.pose.orientation
    ori = numpy.array([ori.x, ori.y, ori.z, ori.w])
    if (base_ori == None):
        base_ori = ori
        angle = 0.0
    else:
        angle = degrees(2 * acos(numpy.dot(base_ori, ori)))
    if angle > 180:
        angle = 360 - angle
    angles.append(angle)
    print angle

def main():
    global angles
    rospy.init_node('relative_angle_node')
    rospy.Subscriber('/outlet_detector/outlet_pose', PoseStamped, pose_cb)
    #rospy.Subscriber('/plug_detector/plug_pose', PoseStamped, pose_cb)
    rospy.spin()
    print '\nReadings: %i' % len(angles)
    angles = numpy.array(angles)
    #print 'Angle:\n\tMean = %f degrees\n\tStd dev = %f' % (angles.mean(), angles.std())

if __name__ == '__main__': main()
