#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id: listener_with_userdata 3059 2008-12-10 00:39:00Z sfkwc $

## Gathers camera, laser, and mechanism state data into chunks

PKG = 'laser_camera_calibration' # this package name
NAME = 'lasercamera_gatherer'

import rostools; rostools.update_path(PKG) 

import sys
import thread
from numpy import *

import rospy
from rostools import rostime
from std_msgs.msg import LaserScan
from robot_msgs.msg import MechanismState
from checkerboard_detector.msg import ObjectDetection
import copy

class GatherData:
    objectthresh = 0.005 # meters + scale * quaternion dist
    publishinterval = rostime.Duration(secs = 0.5)

    def __init__(self, *args):
        self.laserqueue = []
        self.robotqueue = []
        self.objdetqueue = []
        self.lastpublished = rostime.Time().now()
        self.mutex = thread.allocate_lock()
        self.numpublished = 0

        self.pub_objdet = rospy.Publisher("new_ObjectDetection", ObjectDetection)
        self.pub_laser = rospy.Publisher("new_tilt_scan", LaserScan)
        self.pub_robot = rospy.Publisher("new_mechanism_state", MechanismState)
        rospy.Subscriber("ObjectDetection", ObjectDetection, self.callback_objdet, 1)
        rospy.Subscriber("tilt_scan", LaserScan, self.callback_laser, 1)
        rospy.Subscriber("mechanism_state", MechanismState, self.callback_robot, 1)
        rospy.init_node(NAME, anonymous=True)
        
    def pose_dist(self, pose1, pose2):
        t1 = array([pose1.position.x, pose1.position.y, pose1.position.z])
        q1 = array([pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w])
        t2 = array([pose2.position.x, pose2.position.y, pose2.position.z])
        q2 = array([pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w])
        return sqrt(sum(t1-t2)**2) + 0.3 * sqrt(sum(q1-q2)**2)

    def get_neighbors(self, msglist, stamp):
        indices = filter(lambda i: stamp > msglist[i].header.stamp and stamp <= msglist[i+1].header.stamp,range(0,len(msglist)-1))
        if len(indices) == 0:
            return None, None

        return msglist[indices[-1]], msglist[indices[-1]+1]

    def insert_msg(self, msglist, msg, dumpinterval):
        while  len(msglist) > 0:
            # try to reject
            if msg.header.stamp - msglist[0].header.stamp < dumpinterval:
                break
            msglist.pop(0)
        msglist.append(msg)

    def callback_laser(self, lasermsg, arg):
        self.mutex.acquire()
        try:
            self.insert_msg(self.laserqueue,lasermsg, rostime.Duration(secs = 1))
        finally:
            self.mutex.release()

    def callback_robot(self, robotmsg, arg):
        self.mutex.acquire()
        try:
            self.insert_msg(self.robotqueue,robotmsg, rostime.Duration(secs = 0.5))
        finally:
            self.mutex.release()

    def callback_objdet(self, objdetmsg, arg):
        self.mutex.acquire()
        try:
            self.insert_msg(self.objdetqueue, objdetmsg, rostime.Duration(secs = 4))

            # test all lists for publishing
            if rostime.Time().now()-self.lastpublished < self.publishinterval:
                return

            # return if anything other than one object was detected
            if not all([len(x.objects)==1 for x in self.objdetqueue ]):
                return

            # make sure the object type is the same and transformation is close
            obj = self.objdetqueue[-1].objects[0]
            if not all([x.objects[0].type==obj.type for x in self.objdetqueue ]):
                return
            if not all([self.pose_dist(x.objects[0].pose,obj.pose)<=self.objectthresh for x in self.objdetqueue]):
                return

            if len(self.laserqueue) < 2 or len(self.robotqueue) < 2:
                return
            
            stamp = min(self.objdetqueue[-1].header.stamp, self.laserqueue[-1].header.stamp, self.robotqueue[-1].header.stamp)

            # interpolate scanlines, find two scanlines that are between stamp
            laserprev, lasernext = self.get_neighbors(self.laserqueue, stamp)
            objdetprev, objdetnext = self.get_neighbors(self.objdetqueue, stamp)
            robotprev, robotnext = self.get_neighbors(self.robotqueue, stamp)

            if laserprev is None or objdetprev is None or robotprev is None:
                return

            # validate robot values
            jointsprev = array([x.position for x in robotprev.joint_states])
            jointsnext = array([x.position for x in robotnext.joint_states])
            robotdtime = (robotnext.header.stamp-stamp).to_seconds()/(robotnext.header.stamp-robotprev.header.stamp).to_seconds()
            if not len(jointsprev) == len(jointsnext):
                print "joint lengths different"
                return

            # validate laser values
            if not laserprev.angle_min == lasernext.angle_min or not laserprev.angle_max == lasernext.angle_max or not laserprev.angle_increment == lasernext.angle_increment or not laserprev.time_increment == lasernext.time_increment or not laserprev.scan_time == lasernext.scan_time or not laserprev.range_min == lasernext.range_min or not laserprev.range_max == lasernext.range_max:
                return

            rangesprev = array(laserprev.ranges)
            rangesnext = array(lasernext.ranges)
            laserdtime = (lasernext.header.stamp-stamp).to_seconds()/(lasernext.header.stamp-laserprev.header.stamp).to_seconds()
            if not len(rangesprev) == len(rangesnext):
                print "ranges different"
                return

            # validate obj values
            poseprev = objdetprev.objects[0].pose
            posenext = objdetnext.objects[0].pose
            objdetdtime = (objdetnext.header.stamp-stamp).to_seconds()/(objdetnext.header.stamp-objdetprev.header.stamp).to_seconds()

            if robotdtime < 0 or laserdtime < 0 or objdetdtime < 0:
                print "times are negative, skipping"
                return

            joints = jointsprev*robotdtime+jointsnext*(1-robotdtime)
            ranges = rangesprev*laserdtime+rangesnext*(1-laserdtime)
            pose = posenext # pose changes slow so just pick one

            objdetmsg = copy.copy(self.objdetqueue[-1])
            objdetmsg.header.stamp = stamp; objdetmsg.objects[0].pose = pose;
            lasermsg = copy.copy(self.laserqueue[-1]); lasermsg.header.stamp = stamp; lasermsg.ranges = ranges;
            robotmsg = copy.copy(self.robotqueue[-1]); robotmsg.header.stamp = stamp;
            for i in range(len(robotmsg.joint_states)):
                robotmsg.joint_states[i].position = joints[i]

            self.pub_objdet.publish(objdetmsg)
            self.pub_laser.publish(lasermsg)
            self.pub_robot.publish(robotmsg)

            self.numpublished = self.numpublished+1
            print "published ",self.numpublished
        finally:
            self.mutex.release()

if __name__ == '__main__':
    try:
        gatherer = GatherData()
        rospy.spin()
    except KeyboardInterrupt, e:
        pass
    print "exiting"
