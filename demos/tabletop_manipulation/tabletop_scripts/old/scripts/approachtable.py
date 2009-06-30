#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
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
# Revision $Id: rossync 3844 2009-02-16 19:49:10Z gerkey $

import roslib
roslib.load_manifest('tabletop_manipulation')
import rospy
from robot_srvs.srv import FindTable, FindTableRequest
from robot_msgs.msg import Polygon3D, Point
from visualization_msgs.msg import Marker
from deprecated_msgs.msg import RobotBase2DOdom
from tf.msg import tfMessage
import bullet
import tf

from movebase import MoveBase
from detecttable import DetectTable
from tiltscan import TiltScan

import sys
from math import *

class ApproachTable:
  def __init__(self, dt, mb, ts):
    self.ts = ts
    self.mb = mb
    self.dt = dt
    self.odom_pose = None
    self.robot_position = None
    self.vm = None
    rospy.Subscriber('odom', RobotBase2DOdom, self.odomCallback)
    rospy.Subscriber('tf_message', tfMessage, self.tfCallback)
    self.pub_vis = rospy.Publisher("visualization_marker", Marker)
    self.global_frame = rospy.get_param('/global_frame_id')

  # Hack to get around the fact that we don't have pytf
  def tfCallback(self,msg):
    if (msg.transforms[0].header.frame_id == self.global_frame and 
        self.odom_pose != None):
      self.robot_position = Point((self.odom_pose.pos.x - 
                                   msg.transforms[0].transform.translation.x),
                                  (self.odom_pose.pos.y - 
                                   msg.transforms[0].transform.translation.y),
                                  0.0)

  def odomCallback(self,msg):
    self.odom_pose = msg

  def approachTable(self, standoff, near):
    if not ts.tiltScan(8.0):
      print '[ApproachTable] Failed to change tilt scan'
      return False
      
    resp = dt.detectTable()
    if not resp:
      print '[ApproachTable] Failed to detect table'
      return False
  
    self.vm = Marker()
    self.vm.header.frame_id = resp.table.header.frame_id
    self.vm.ns = "approachtable";
    self.vm.pose.position.z = resp.table.table.points[0].z
  
    approach_pose = self.computeApproachPose(self.robot_position, resp.table.table, standoff, True)
    
    if approach_pose == None:
      return False
  
    self.vm.id = 1000
    self.vm.type = Marker.ARROW
    self.vm.action = Marker.ADD
    self.vm.pose.position.x = approach_pose[0]
    self.vm.pose.position.y = approach_pose[1]
    #self.vm.z = 0.0
    self.vm.pose.orientation = tf.quaternion_bt_to_msg(bullet.Quaternion(approach_pose[2], 0.0, 0.0))
    self.vm.scale.x = 0.6
    self.vm.scale.y = 0.25
    self.vm.scale.z = 0.1
    self.vm.color.a = 0.5
    self.vm.color.r = 1.0
    self.vm.points = []
    self.pub_vis.publish(self.vm)

    if not ts.tiltScan(2.0):
      print '[ApproachTable] Failed to change tilt scan'
      return False

    # Call out to blocking MoveBase
    return self.mb.moveBase(resp.table.header.frame_id,
                            approach_pose[0],
                            approach_pose[1],
                            approach_pose[2])
  
  def computeApproachPose(self, pose, poly, d, near):
    if not near:
      print '[ApproachTable] Far approach not yet implemented'
      sys.exit(-1)

  
    if pose == None:
      print '[ApproachTable] No robot pose!'
      return

    # TODO: implement approaching from the far side (not near)
  
    # Find the two closest vertices
  
    # First find the closest one
    min_sqd = [-1.0, -1.0]
    closestp = [None, None]
    for p in poly.points:
      sqd = (pose.x - p.x)*(pose.x - p.x) + \
            (pose.y - p.y)*(pose.y - p.y)
      if min_sqd[0] < 0.0 or sqd < min_sqd[0]:
        min_sqd[0] = sqd
        closestp[0] = p
  
    if closestp[0] == None:
      print '[ApproachTable] No closest point!'
      return
  
    # Now find the second-closest, but require that it be some distance from
    # the first one
    for p in poly.points:
      sqd = (pose.x - p.x)*(pose.x - p.x) + \
            (pose.y - p.y)*(pose.y - p.y)
      if min_sqd[1] < 0.0 or sqd < min_sqd[1]:
        sqd2 = (closestp[0].x - p.x)*(closestp[0].x - p.x) + \
               (closestp[0].y - p.y)*(closestp[0].y - p.y)
        if sqd2 > 0.25:
          min_sqd[1] = sqd
          closestp[1] = p

    if closestp[1] == None:
      print '[ApproachTable] No second-closest point!'
      return
  
    # Also find the farthest, because we'll do a poor-man's point-in-polygon
    # check
    farthestp = None
    maxsqd = -1.0
    for p in poly.points:
      sqd = (pose.x - p.x)*(pose.x - p.x) + \
            (pose.y - p.y)*(pose.y - p.y)
      if maxsqd < 0.0 or sqd > maxsqd:
        maxsqd = sqd
        farthestp = p
  
    if farthestp == None:
      print '[ApproachTable] No farthest point!'
      return
  
    print '[ApproachTable] Robot pose: (%f,%f)'%(pose.x,pose.y)
    print '[ApproachTable] Closest points: (%f,%f,%f),(%f,%f,%f)'%(closestp[0].x,closestp[0].y,closestp[0].z,closestp[1].x,closestp[1].y,closestp[1].z)
  
    # Find the midpoint of the closest edge
    midp = Point(closestp[0].x + (closestp[1].x - closestp[0].x)/2.0,
                 closestp[0].y + (closestp[1].y - closestp[0].y)/2.0, 0.0)
  
    # The angle of the edge
    theta = atan2(closestp[1].y-closestp[0].y, closestp[1].x-closestp[0].x)
  
    # Which direction is toward the robot?
    theta2 = atan2((farthestp.y-closestp[0].y),(farthestp.x-closestp[0].x))
    dir = theta2-theta
    dir = atan2(sin(dir),cos(dir))
    if dir > 0:
      phi = theta - pi/2.0
    else:
      phi = theta + pi/2.0
  
    # Add in standoff distance
    goalx = midp.x + d * cos(phi)
    goaly = midp.y + d * sin(phi)
    # We want to point at the table
    goala = phi + pi
    goala = atan2(sin(goala),cos(goala))
  
    self.vm.id = 1001
    self.vm.type = Marker.SPHERE
    self.vm.action = Marker.ADD
    self.vm.pose.position.x = midp.x
    self.vm.pose.position.y = midp.y
    #self.vm.z = 0.0
    self.vm.pose.orientation.x = 0.0
    self.vm.pose.orientation.y = 0.0
    self.vm.pose.orientation.z = 0.0
    self.vm.pose.orientation.w = 1.0
    self.vm.scale.x = 0.05
    self.vm.scale.y = 0.05
    self.vm.scale.z = 0.05
    self.vm.color.a = 0.5
    self.vm.color.r = 0.0
    self.vm.color.g = 0.0
    self.vm.color.b = 1.0
    self.vm.points = []
    self.pub_vis.publish(self.vm)
  
    self.vm.id = 1002
    self.vm.type = Marker.SPHERE
    self.vm.action = Marker.ADD
    self.vm.pose.position.x = closestp[0].x
    self.vm.pose.position.y = closestp[0].y
    #self.vm.z = 0.0
    self.vm.color.r = 0.0
    self.vm.color.g = 1.0
    self.vm.color.b = 0.0
    self.vm.points = []
    self.pub_vis.publish(self.vm)
  
    self.vm.id = 1003
    self.vm.type = Marker.SPHERE
    self.vm.action = Marker.ADD
    self.vm.pose.position.x = closestp[1].x
    self.vm.pose.position.y = closestp[1].y
    #self.vm.z = 0.0
    self.vm.color.r = 1.0
    self.vm.color.g = 0.0
    self.vm.color.b = 1.0
    self.vm.points = []
    self.pub_vis.publish(self.vm)
  
    self.vm.id = 1004
    self.vm.type = Marker.SPHERE
    self.vm.action = Marker.ADD
    self.vm.pose.position.x = farthestp.x
    self.vm.pose.position.y = farthestp.y
    #self.vm.z = 0.0
    self.vm.color.r = 1.0
    self.vm.color.g = 01
    self.vm.color.b = 1.0
    self.vm.points = []
    self.pub_vis.publish(self.vm)
  
    print '[ApproachTable] midp: %f,%f'%(midp.x,midp.y)
    print '[ApproachTable] theta: %f phi:%f' %(theta*180.0/pi,phi*180.0/pi)
    print '[ApproachTable] Goal: %f,%f,%f'%(goalx, goaly, goala*180.0/pi)
  
    return (goalx, goaly, goala)
  
if __name__ == '__main__':
  ts = TiltScan('laser_tilt_controller', 5.0)
  mb = MoveBase()
  dt = DetectTable()
  at = ApproachTable(dt, mb, ts)

  rospy.init_node('approach_table', anonymous=True)

  res = False
  res = at.approachTable(0.4, True)
  if not res:
    if at.approachTable(1.0, True):
      res = at.approachTable(0.4, True)

  if not res:
    print 'Failure!'
  else:
    print 'Success!'
    
