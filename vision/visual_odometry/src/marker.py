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

import roslib
roslib.load_manifest('visual_odometry')

from robot_msgs.msg import Point
from visualization_msgs.msg import Marker
import rospy
from visualodometer import Pose

import numpy
from math import pi

vm_pub = rospy.Publisher("/visualization_marker", Marker)

tableh = 2.5

class Marker:
  def __init__(self, uniqid):
    self.id = uniqid

  def floor(self):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.ns = "visual_odometry"
    marker.id = self.id + 99999
    marker.type = 1
    marker.action = 0
    marker.pose.position.z = -(5 + tableh + .1)
    marker.pose.orientation.w = 1.0
    marker.scale.x = 10
    marker.scale.y = 10
    marker.scale.z = 10
    marker.color.a = 1.0
    marker.color.r = 0.3
    marker.color.g = 0.4
    marker.color.b = 0.7
    marker.points = []
    vm_pub.publish(marker)

  def update(self, offset, x, y, z, r, color):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.ns = "visual_odometry"
    marker.id = self.id + offset
    marker.type = 2
    marker.action = 0
    marker.pose.position.x = x
    marker.pose.position.y = y
    marker.pose.position.z = z
    marker.pose.orientation.w = 1.0
    marker.scale.x = marker.scale.y = marker.scale.z = r
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.points = []
    vm_pub.publish(marker)
    marker.scale.z = 0
    marker.pose.position.z = -tableh
    marker.id += 200
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    vm_pub.publish(marker)

  def linestrip(self, offset, points, color):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.ns = "visual_odometry"
    marker.id = self.id + offset
    marker.type = 4
    marker.action = 0
    marker.pose.orientation.w = 1.0
    marker.scale.x = 0.1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.points = points
    vm_pub.publish(marker)

    marker.points = [Point(p.x, p.y, -tableh) for p in points]
    marker.id += 200
    marker.color.r = 0
    marker.color.g = 0
    marker.color.b = 0
    vm_pub.publish(marker)

  def frompose(self, ipose, cam, color):
    factor = 1e-1

    xf = Pose(numpy.array([[ 1, 0, 0], [0, 0, 1], [0, -1, 0]]), [0, 0, 0])
    pose = xf*ipose

    #print ipose.xform(0,0,0), pose.xform(0,0,0)

    for xi,xoff in [ (0,0), (100, 0.088) ]:
      x,y,z = pose.xform(xoff,0,0)
      x /= factor
      y /= factor
      z /= factor
      self.update(xi, x,y,z, .4, color)
      x,y,z = pose.xform(xoff,0,factor)
      x /= factor
      y /= factor
      z /= factor
      self.update(xi+1, x,y,z, 0.1, color)

      def mod2pos(x,y,z):
        x,y,z = pose.xform(x+xoff, y, z)
        x /= factor
        y /= factor
        z /= factor
        return Point(x,y,z)
        
      for i,(x,y) in enumerate([ (0,0), (640,0), (0,480), (640,480)]):
        model = [(0,0,0), cam.pix2cam(x, y, 640)]
        self.linestrip(xi+2+i, [mod2pos(*p) for p in model], color)
