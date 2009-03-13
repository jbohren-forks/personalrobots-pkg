#!/usr/bin/env python
import roslib; roslib.load_manifest('outlet_detection')

import rospy
from robot_msgs.msg import *
import threading
import numpy
from math import acos, degrees

outlet_lock = threading.Lock()
outlet_pose = None
plug_pose = None

def outlet_cb(data):
    global outlet_lock, outlet_pose

    outlet_lock.acquire()
    outlet_pose = data.pose
    outlet_lock.release()

def plug_cb(data):
    global outlet_lock, outlet_pose, plug_pose

    plug_pose = data.pose
    plug_pos = plug_pose.position
    plug_pos = numpy.array([plug_pos.x, plug_pos.y, plug_pos.z])
    plug_ori = plug_pose.orientation
    plug_ori = numpy.array([plug_ori.x, plug_ori.y, plug_ori.z, plug_ori.w])

    outlet_lock.acquire()

    if outlet_pose:
        outlet_pos = outlet_pose.position
        outlet_pos = numpy.array([outlet_pos.x, outlet_pos.y, outlet_pos.z])
        outlet_ori = outlet_pose.orientation
        outlet_ori = numpy.array([outlet_ori.x, outlet_ori.y, outlet_ori.z, outlet_ori.w])

        distance = 1000 * numpy.linalg.norm(plug_pos - outlet_pos)
        #print "Dot product = %f" % numpy.dot(plug_ori, outlet_ori)
        angle = degrees(2 * acos(numpy.dot(plug_ori, outlet_ori)))
        print "Distance = %.2fmm, angle = %.2f degrees" % (distance, angle)
    
    outlet_lock.release()

def main():
    rospy.init_node('relative_pose_plug_outlet')
    rospy.Subscriber('/outlet_detector/pose', PoseStamped, outlet_cb)
    rospy.Subscriber('/plug_detector/pose', PoseStamped, plug_cb)
    rospy.spin()

if __name__ == '__main__': main()
