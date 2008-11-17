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

import rostools
rostools.update_path('visual_odometry')

from std_msgs.msg import VisualizationMarker, Position
import rospy
from visualodometer import Pose

import numpy
from math import pi

vm_pub = rospy.Publisher("/visualizationMarker", VisualizationMarker)

tableh = 2.5

class Marker:
  def __init__(self, uniqid):
    self.id = uniqid

  def floor(self):
    marker = VisualizationMarker()
    marker.header.frame_id = "base"
    marker.id = self.id + 99999
    marker.type = 1
    marker.action = 0
    marker.x = 0
    marker.y = 0
    marker.z = -(5 + tableh + .1)
    marker.yaw = 0
    marker.pitch = 0
    marker.roll = 0
    marker.xScale = 10
    marker.yScale = 10 
    marker.zScale = 10
    marker.alpha = 255
    marker.r = 0.3 * 255
    marker.g = 0.4 * 255
    marker.b = 0.7 * 255
    marker.points = []
    vm_pub.publish(marker)
    print "sending floor"

  def update(self, offset, x, y, z, r, color):
    marker = VisualizationMarker()
    marker.header.frame_id = "base"
    marker.id = self.id + offset
    marker.type = 2
    marker.action = 0
    marker.x = x
    marker.y = y
    marker.z = z
    marker.yaw = 0
    marker.pitch = 0
    marker.roll = 0
    marker.xScale = r
    marker.yScale = r
    marker.zScale = r
    marker.alpha = 255
    marker.r = color[0]
    marker.g = color[1]
    marker.b = color[2]
    marker.points = []
    vm_pub.publish(marker)
    marker.zScale = 0
    marker.z = -tableh
    marker.id += 200
    marker.r = 0
    marker.g = 0
    marker.b = 0
    vm_pub.publish(marker)

  def linestrip(self, offset, points, color):
    marker = VisualizationMarker()
    marker.header.frame_id = "base"
    marker.id = self.id + offset
    marker.type = 4
    marker.action = 0
    marker.x = 0
    marker.y = 0
    marker.z = 0
    marker.yaw = 0
    marker.pitch = 0
    marker.roll = 0
    marker.xScale = .1
    marker.yScale = 1
    marker.zScale = 1
    marker.alpha = 255
    marker.r = color[0]
    marker.g = color[1]
    marker.b = color[2]
    marker.points = points
    vm_pub.publish(marker)

    marker.points = [Position(p.x, p.y, -tableh) for p in points]
    marker.id += 200
    marker.r = 0
    marker.g = 0
    marker.b = 0
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
        return Position(x,y,z)
        
      for i,(x,y) in enumerate([ (0,0), (640,0), (0,480), (640,480)]):
        model = [(0,0,0), cam.pix2cam(x, y, 640)]
        self.linestrip(xi+2+i, [mod2pos(*p) for p in model], color)
