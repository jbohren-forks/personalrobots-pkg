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
NAME = 'set_plug_on_base'

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

xyz = [0,0,0]
rpy = [0,0,0]
test_timeout = 5000;
tolerance = 0.01;
magnetic_cutoff = 0.01;
goal_reached = False
in_magnet_range = True;

x_origin = 0.0
y_origin = 0.0
z_origin = 0.0

magnet_p = [0,0,0]
magnet_e = [0,0,0]
magnet_q = [1,0,0,0]

def  plugP3DInput(p3d):
    global xyz,rpy,goal_reached
    if not goal_reached:
      p3d_x = p3d.pos.position.x
      p3d_y = p3d.pos.position.y
      p3d_z = p3d.pos.position.z
      p3d_qw = p3d.pos.orientation.w
      p3d_qx = p3d.pos.orientation.x
      p3d_qy = p3d.pos.orientation.y
      p3d_qz = p3d.pos.orientation.z

      # check plug position from goal
      e = euler_from_quaternion([p3d_qx,p3d_qy,p3d_qz,p3d_qw])
      if tolerance > 0:
        if abs(p3d_x - xyz[0]) < tolerance and \
           abs(p3d_y - xyz[1]) < tolerance and \
           abs(p3d_z - xyz[2]) < tolerance and \
           abs(rpy[0] - e[0])  < tolerance and \
           abs(rpy[1] - e[1])  < tolerance and \
           abs(rpy[2] - e[2])  < tolerance:
            goal_reached = True

      if magnetic_cutoff > 0:
        if abs(p3d_x - xyz[0]) > magnetic_cutoff and \
           abs(p3d_y - xyz[1]) > magnetic_cutoff and \
           abs(p3d_z - xyz[2]) > magnetic_cutoff:
            in_magnet_range = False

def  magnetP3DInput(p3d):
    global xyz,rpy,goal_reached,magnet_p,magnet_q,magnet_e
    if not goal_reached:
      magnet_p = [p3d.pos.position.x - x_origin, p3d.pos.position.y - y_origin, p3d.pos.position.z - z_origin]
      magnet_q = [p3d.pos.orientation.x, p3d.pos.orientation.y, p3d.pos.orientation.z, p3d.pos.orientation.w]
      magnet_e = euler_from_quaternion([magnet_q[0],magnet_q[1],magnet_q[2],magnet_q[3]])

def main():
    global xyz, rpy, test_timeout, goal_reached

    # get goal from commandline
    for i in range(0,len(sys.argv)):
      if sys.argv[i] == '-timeout':
        if len(sys.argv) > i+1:
          test_timeout = float(sys.argv[i+1])
      if sys.argv[i] == '-xyz':
        if len(sys.argv) > i+3:
          xyz = [float(sys.argv[i+1]), float(sys.argv[i+2]), float(sys.argv[i+3])]
      if sys.argv[i] == '-rpy':
        if len(sys.argv) > i+3:
          rpy = [float(sys.argv[i+1]), float(sys.argv[i+2]), float(sys.argv[i+3])]
      if sys.argv[i] == '-mag':
        if len(sys.argv) > i+1:
          magnetic_cutoff = float(sys.argv[i+1])
      if sys.argv[i] == '-tol':
        if len(sys.argv) > i+1:
          tolerance = float(sys.argv[i+1])

    pub_pose = rospy.Publisher("set_plug_pose", PoseWithRatesStamped)
    rospy.Subscriber("plug_pose_ground_truth", PoseWithRatesStamped, plugP3DInput)
    rospy.Subscriber("plug_magnet_pose_ground_truth", PoseWithRatesStamped, magnetP3DInput)

    rospy.init_node(NAME, anonymous=True)

    timeout_t = time.time() + test_timeout

    # wait for result
    while not goal_reached and not rospy.is_shutdown() and time.time() < timeout_t:
      if in_magnet_range:
        #create a temp header for publishers
        h = rospy.Header();
        h.stamp = rospy.get_rostime();
        h.frame_id = "map"
        # publish pose
        p = Point(xyz[0]+magnet_p[0],xyz[1]+magnet_p[1],xyz[2]+magnet_p[2])
        tmpq = quaternion_multiply(quaternion_from_euler(rpy[0],rpy[1],rpy[2],'rxyz'),magnet_q)
        q = Quaternion(tmpq[0],tmpq[1],tmpq[2],tmpq[3])
        pose = Pose(p,q)
        poseWithRatesStamped = PoseWithRatesStamped(h,pose,PoseDot(),PoseDDot());
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


