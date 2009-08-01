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

## Gazebo send plug position topic for ros_sim_iface consumption

PKG = 'plug_in_gazebo'
NAME = 'set_plug'

import math
import roslib
roslib.load_manifest(PKG)

import sys, unittest
import os, os.path, threading, time
import rospy, rostest
from std_msgs.msg import *
from robot_actions.msg import *
from nav_robot_actions.msg import *
from robot_msgs.msg import *
from tf.transformations import *
from geometry_msgs.msg import Twist, PoseWithRatesStamped
from numpy import *

def normalize_angle_positive(angle):
    return math.fmod(math.fmod(angle, 2*math.pi) + 2*math.pi, 2*math.pi)

def normalize_angle(angle):
    anorm = normalize_angle_positive(angle)
    if anorm > math.pi:
      anorm -= 2*math.pi
    return anorm

def shortest_angular_distance(angle_from, angle_to):
    angle_diff = normalize_angle_positive(angle_to) - normalize_angle_positive(angle_from)
    if angle_diff > math.pi:
      angle_diff = -(2*math.pi - angle_diff)
    return normalize_angle(angle_diff)

xyz_x = 0;
xyz_y = 0;
xyz_z = 0;
rot_r = 0;
rot_p = 0;
rot_y = 0;
test_timeout = 50;
tolerance = 0.01;
object_moved = False

def p3dInput(p3d):
    global xyz_x,xyz_y,xyz_z,rot_r,rot_p,rot_y,object_moved
    if not object_moved:
      p3d_x = p3d.pos.position.x
      p3d_y = p3d.pos.position.y
      p3d_z = p3d.pos.position.z
      p3d_rw = p3d.pos.orientation.w
      p3d_rx = p3d.pos.orientation.x
      p3d_ry = p3d.pos.orientation.y
      p3d_rz = p3d.pos.orientation.z

      # check plug position from goal
      e = euler_from_quaternion([p3d_rx,p3d_ry,p3d_rz,p3d_rw])
      if tolerance > 0:
        if abs(p3d_x - xyz_x) < tolerance and \
           abs(p3d_y - xyz_y) < tolerance and \
           abs(p3d_z - xyz_z) < tolerance and \
           abs(rot_r - e[0])  < tolerance and \
           abs(rot_p - e[1])  < tolerance and \
           abs(rot_y - e[2])  < tolerance:
            object_moved = True

def main():
    global xyz_x, xyz_y, xyz_z, rot_r, rot_p, rot_y, test_timeout, object_moved

    # get goal from commandline
    for i in range(0,len(sys.argv)):
      if sys.argv[i] == '-timeout':
        if len(sys.argv) > i+1:
          test_timeout = float(sys.argv[i+1])
      if sys.argv[i] == '-xyz':
        if len(sys.argv) > i+3:
          xyz_x = float(sys.argv[i+1])
          xyz_y = float(sys.argv[i+2])
          xyz_z = float(sys.argv[i+3])
      if sys.argv[i] == '-rpy':
        if len(sys.argv) > i+3:
          rot_r = float(sys.argv[i+1])
          rot_p = float(sys.argv[i+2])
          rot_y = float(sys.argv[i+3])
      if sys.argv[i] == '-tol':
        if len(sys.argv) > i+1:
          tolerance = float(sys.argv[i+1])

    pub_pose = rospy.Publisher("set_plug_pose", PoseWithRatesStamped)
    rospy.Subscriber("plug_pose_ground_truth", PoseWithRatesStamped, p3dInput)

    rospy.init_node(NAME, anonymous=True)

    timeout_t = time.time() + test_timeout

    # wait for result
    while not object_moved and not rospy.is_shutdown() and time.time() < timeout_t:
        #create a temp header for publishers
        h = rospy.Header();
        h.stamp = rospy.get_rostime();
        h.frame_id = "map"
        # publish pose
        p = Point(xyz_x,xyz_y,xyz_z)
        tmpq = quaternion_from_euler(rot_r,rot_p,rot_y,'rxyz')
        q = Quaternion(tmpq[0],tmpq[1],tmpq[2],tmpq[3])
        pose = Pose(p,q)
        poseWithRatesStamped = PoseWithRatesStamped(h,pose,Twist(),Twist());
        pub_pose.publish(poseWithRatesStamped)
        time.sleep(0.05)

def print_usage(exit_code = 0):
    print '''Commands:
    -timeout <seconds> - test timeout in seconds. default to 50 seconds
    -xyz <x> <y> <z>
    -rpy <roll> <pitch> <yaw>
'''

if __name__ == '__main__':
    #print usage if not arguments
    if len(sys.argv) == 1:
      print_usage()
    else:
      main()


