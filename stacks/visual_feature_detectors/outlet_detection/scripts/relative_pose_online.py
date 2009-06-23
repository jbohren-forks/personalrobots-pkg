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
distances = []
angles = []

def outlet_cb(data):
    global outlet_lock, outlet_pose

    outlet_lock.acquire()
    outlet_pose = data.pose
    outlet_lock.release()

def plug_cb(data):
    global outlet_lock, outlet_pose, plug_pose, distances, angles

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
        angle = degrees(2 * acos(numpy.dot(plug_ori, outlet_ori)))
        if angle > 40:
            print "Distance = %.2fmm, angle = %.2f degrees, REJECTED" % (distance, angle)
        else:
            distances.append(distance)
            angles.append(angle)
            print "Distance = %.2fmm, angle = %.2f degrees" % (distance, angle)
    
    outlet_lock.release()

def main():
    global distances, angles
    rospy.init_node('relative_pose_plug_outlet')
    rospy.Subscriber('/outlet_detector/outlet_pose', PoseStamped, outlet_cb)
    rospy.Subscriber('/plug_detector/plug_pose', PoseStamped, plug_cb)
    rospy.spin()
    print '\nReadings: %i' % len(distances)
    distances = numpy.array(distances)
    angles = numpy.array(angles)
    print 'Distance:\n\tMean = %fmm\n\tStd dev = %f' % (distances.mean(), distances.std())
    print 'Angle:\n\tMean = %f degrees\n\tStd dev = %f' % (angles.mean(), angles.std())

if __name__ == '__main__': main()
