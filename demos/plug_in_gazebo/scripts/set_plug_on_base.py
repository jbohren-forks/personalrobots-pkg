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


from std_msgs.msg import String
from nav_robot_actions.msg import MoveBaseState
from geometry_msgs.msg import Pose,Quaternion,Point, PoseStamped, PoseWithCovariance, TwistWithCovariance, Wrench, Vector3
from nav_msgs.msg import Odometry
import tf.transformations as tft
from numpy import float64

# simulate magnetic forces between plug and holder.
# simulate magnetic forces between plug and gripper_tool_frame.

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

update_rate = 100;
timeout = 5000;

holder_cutoff = 0.03;
in_holder_range = True;

holder_p = [0,0,0]
holder_q = [1,0,0,0]
holder_e = [0,0,0]

gripper_cutoff = 0.03;
in_gripper_range = False;

gripper_p = [0,0,0]
gripper_q = [1,0,0,0]
gripper_e = [0,0,0]

plug_p = [0,0,0]
plug_q = [1,0,0,0]
plug_e = [0,0,0]

linear_gain_p = 1.0
angular_gain_p = 1.0

plug_position_initialized = False
holder_position_initialized = False
gripper_position_initialized = False

def  plugP3DInput(p3d):
    global force,torque,holder_p,holder_e, holder_position_initialized, force,torque,plug_p,plug_e, plug_position_initialized, in_holder_range, holder_cutoff, linear_gain_p, angular_gain_p, timeout, update_rate, gripper_p, gripper_q, gripper_e, in_gripper_range, gripper_cutoff, gripper_position_initialized, plug_q, holder_q, gripper_q

    plug_p = [p3d.pose.pose.position.x, p3d.pose.pose.position.y, p3d.pose.pose.position.z]
    plug_q = [p3d.pose.pose.orientation.x, p3d.pose.pose.orientation.y, p3d.pose.pose.orientation.z, p3d.pose.pose.orientation.w]
    plug_e = tft.euler_from_quaternion(plug_q)

    plug_position_initialized = True

def  holderP3DInput(p3d):
    global force,torque,holder_p,holder_e, holder_position_initialized, force,torque,plug_p,plug_e, plug_position_initialized, in_holder_range, holder_cutoff, linear_gain_p, angular_gain_p, timeout, update_rate, gripper_p, gripper_q, gripper_e, in_gripper_range, gripper_cutoff, gripper_position_initialized, plug_q, holder_q, gripper_q
    holder_p = [p3d.pose.pose.position.x, p3d.pose.pose.position.y, p3d.pose.pose.position.z]
    holder_q = [p3d.pose.pose.orientation.x, p3d.pose.pose.orientation.y, p3d.pose.pose.orientation.z, p3d.pose.pose.orientation.w]
    holder_e = tft.euler_from_quaternion(holder_q)

    holder_position_initialized = True

def  gripperP3DInput(p3d):
    global force,torque,holder_p,holder_e, holder_position_initialized, force,torque,plug_p,plug_e, plug_position_initialized, in_holder_range, holder_cutoff, linear_gain_p, angular_gain_p, timeout, update_rate, gripper_p, gripper_q, gripper_e, in_gripper_range, gripper_cutoff, gripper_position_initialized, plug_q, holder_q, gripper_q
    gripper_p = [p3d.pose.pose.position.x, p3d.pose.pose.position.y, p3d.pose.pose.position.z]
    gripper_q = [p3d.pose.pose.orientation.x, p3d.pose.pose.orientation.y, p3d.pose.pose.orientation.z, p3d.pose.pose.orientation.w]
    gripper_e = tft.euler_from_quaternion(gripper_q)

    gripper_position_initialized = True

def main():
    global force,torque,holder_p,holder_e, holder_position_initialized, force,torque,plug_p,plug_e, plug_position_initialized, in_holder_range, holder_cutoff, linear_gain_p, angular_gain_p, timeout, update_rate, gripper_p, gripper_q, gripper_e, in_gripper_range, gripper_cutoff, gripper_position_initialized, plug_q, holder_q, gripper_q

    # get goal from commandline
    for i in range(0,len(sys.argv)):
      if sys.argv[i] == '-update_rate':
        if len(sys.argv) > i+1:
          update_rate = float(sys.argv[i+1])
      if sys.argv[i] == '-timeout':
        if len(sys.argv) > i+1:
          timeout = float(sys.argv[i+1])
      if sys.argv[i] == '-cutoff_dist':
        if len(sys.argv) > i+1:
          holder_cutoff = float(sys.argv[i+1])
      if sys.argv[i] == '-linear_p':
        if len(sys.argv) > i+1:
          linear_gain_p = float(sys.argv[i+1])
      if sys.argv[i] == '-angular_p':
        if len(sys.argv) > i+1:
          angular_gain_p = float(sys.argv[i+1])

    pub_wrench = rospy.Publisher("/plug_force", Wrench)
    rospy.Subscriber("/plug_pose_ground_truth", Odometry, plugP3DInput)
    rospy.Subscriber("/plug_holder_pose_ground_truth", Odometry, holderP3DInput)
    rospy.Subscriber("/r_gripper_tool_frame_pose_ground_truth", Odometry, gripperP3DInput)

    rospy.init_node(NAME, anonymous=True)

    timeout_t = time.time() + timeout

    # apply forces
    while not rospy.is_shutdown() and time.time() < timeout_t:
      if holder_position_initialized and plug_position_initialized and gripper_position_initialized:
        holder_linear_offset  = [ holder_p[0] - plug_p[0], holder_p[1] - plug_p[1], holder_p[2] - plug_p[2] ]
        tmpiq = tft.quaternion_inverse(plug_q)
        tmpdq = tft.quaternion_multiply(holder_q,tmpiq)
        holder_angular_offset = tft.euler_from_quaternion(tmpdq)
        holder_dist = (holder_linear_offset[0]*holder_linear_offset[0] + holder_linear_offset[1]*holder_linear_offset[1] + holder_linear_offset[2]*holder_linear_offset[2])

        tmpdq = tft.quaternion_multiply(plug_q,gripper_q)
        tmpiq = tft.quaternion_inverse(tmpdq)
        tmpe = tft.euler_from_quaternion(tmpiq)
        gripper_linear_offset  = [ gripper_p[0] - plug_p[0], gripper_p[1] - plug_p[1], gripper_p[2] - plug_p[2] ]
        gripper_angular_offset = [ tmpe[0], tmpe[1], tmpe[2] ]
        gripper_dist = (gripper_linear_offset[0]*gripper_linear_offset[0] + gripper_linear_offset[1]*gripper_linear_offset[1] + gripper_linear_offset[2]*gripper_linear_offset[2])

        #print "plug_q ",plug_q
        #print "holder_q ",holder_q
        if holder_dist <= holder_cutoff*holder_cutoff and holder_dist < gripper_dist:
          in_holder_range = True
          in_gripper_range = False
        else:
          if gripper_dist < gripper_cutoff*gripper_cutoff:
            in_holder_range = False
            in_gripper_range = True
        if in_holder_range:
          linear_error  = [ linear_gain_p*holder_linear_offset[0]/max(abs(holder_linear_offset[0]*holder_linear_offset[0]*holder_linear_offset[0]),0.01), \
                            linear_gain_p*holder_linear_offset[1]/max(abs(holder_linear_offset[1]*holder_linear_offset[1]*holder_linear_offset[1]),0.01), \
                            linear_gain_p*holder_linear_offset[2]/max(abs(holder_linear_offset[2]*holder_linear_offset[2]*holder_linear_offset[2]),0.01) ]
          angular_error  = [ angular_gain_p*holder_angular_offset[0]/max(abs(holder_angular_offset[0]*holder_angular_offset[0]*holder_angular_offset[0]),0.1), \
                             angular_gain_p*holder_angular_offset[1]/max(abs(holder_angular_offset[1]*holder_angular_offset[1]*holder_angular_offset[1]),0.1), \
                             angular_gain_p*holder_angular_offset[2]/max(abs(holder_angular_offset[2]*holder_angular_offset[2]*holder_angular_offset[2]),0.1) ]
        else:
          if in_gripper_range:
            linear_error  = [ linear_gain_p*gripper_linear_offset[0]/max(abs(gripper_linear_offset[0]*gripper_linear_offset[0]*gripper_linear_offset[0]),0.01), \
                              linear_gain_p*gripper_linear_offset[1]/max(abs(gripper_linear_offset[1]*gripper_linear_offset[1]*gripper_linear_offset[1]),0.01), \
                              linear_gain_p*gripper_linear_offset[2]/max(abs(gripper_linear_offset[2]*gripper_linear_offset[2]*gripper_linear_offset[2]),0.01) ]
            angular_error  = [0,0,0]
          else:
            linear_error  = [0,0,0]
            angular_error  = [0,0,0]

        # publish wrench
        wrench = Wrench(Vector3(linear_error[0],linear_error[1],linear_error[2]),Vector3(angular_error[0],angular_error[1],angular_error[2]))
        pub_wrench.publish(wrench)
      if update_rate > 0:
        time.sleep(1.0/update_rate)
      else:
        time.sleep(0.001)

def print_usage(exit_code = 0):
    print '''Commands:
    -timeout <seconds> - test timeout in seconds. default to 5000 seconds
    -xyz <x> <y> <z>
    -rpy <roll> <pitch> <yaw>
'''

if __name__ == '__main__':
    #print usage if not arguments
    if len(sys.argv) == 1:
      print_usage()
    else:
      main()


