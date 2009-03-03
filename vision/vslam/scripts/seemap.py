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

import roslib
roslib.load_manifest('vslam')
import rostest
import rospy

import vop

import sys

import visual_odometry as VO
import Image as Image
import ImageChops as ImageChops
import ImageDraw as ImageDraw
import ImageFilter as ImageFilter

import random
import unittest
import math
import copy
import pickle

from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
from skeleton import Skeleton
from reader import reader

from math import *

import camera
import pylab
import numpy

stereo_cam = camera.Camera((389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95))

def render(filename, color, theta, dest):
  skel = Skeleton(stereo_cam)
  skel.load(filename)
  skel.optimize()
  skel.optimize()

  skel.plot(color, False, theta)

  xlim = pylab.xlim()
  ylim = pylab.ylim()
  xrange = xlim[1] - xlim[0]
  yrange = ylim[1] - ylim[0]
  r = max(xrange, yrange) * 0.48
  mid = sum(xlim) / 2
  pylab.xlim(mid - r, mid + r)
  mid = sum(ylim) / 2
  pylab.ylim(mid - r, mid + r)

if len(sys.argv) > 1:
  render(sys.argv[1], 'blue', 0.0, "foo.eps")
  pylab.legend()
  pylab.savefig("foo.eps")
  pylab.show()

if False:
  fig = pylab.figure(figsize=(10,10), linewidth = 0.0)

  im = Image.open("willow-full-0.025.pgm")
  imx = im.size[0] * 0.025
  imy = im.size[1] * 0.025

  pylab.imshow(numpy.fromstring(im.tostring(), numpy.uint8).reshape(im.size[1],im.size[0]), cmap=pylab.cm.gray, extent = (0, imx, 0, imy), interpolation = 'nearest')

  s = fig.add_subplot(111)
  s.set_position( [ 0.05, 0.05, 0.90, 0.90 ] )
  #render("/u/jamesb/ABCDEF", "", pi + 0.1, "ABCDEF.eps")
  def xform(xs, ys):
    scale = 1.07
    xs *= scale
    ys *= scale

    f = pi + 0.13
    xp = xs * cos(f) - ys * sin(f)
    yp = ys * cos(f) + xs * sin(f)
    yp *= 1.05
    return (xp + 14.1 - 1,yp + 26.5 + 1.5)

  render("/u/jamesb/ABCDEF", "", xform, "ABCDEF.eps")

  pylab.legend()
  pylab.savefig("ABCDEF.eps")
  pylab.show()

if True:
  fig = pylab.figure(figsize=(10,10), linewidth = 0.0)

  s = fig.add_subplot(221)
  s.set_position( [ 0.05, 0.55, 0.40, 0.40 ] )
  render("B/mkplot_snap", 'red', pi + -3.14 / 4, "B.eps")

  s = fig.add_subplot(222)
  s.set_position( [ 0.55, 0.55, 0.40, 0.40 ] )
  render("C/mkplot_snap", 'magenta', pi + -0.3, "C.eps")

  s = fig.add_subplot(223)
  s.set_position( [ 0.05, 0.05, 0.40, 0.40 ] )
  render("D/mkplot_snap", 'cyan', pi + -0.2 + 3.14, "D.eps")

  s = fig.add_subplot(224)
  s.set_position( [ 0.55, 0.05, 0.40, 0.40 ] )
  render("E/mkplot_snap", 'darkorange', pi + 3.14 * 0.30, "E.eps")

  #render("F/mkplot_snap", 'brown', 0.0, "F.eps")

  #pylab.legend()
  pylab.savefig("BCDE.eps")
  pylab.show()
