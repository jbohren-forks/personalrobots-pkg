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

import random
import unittest
import math
import copy
import pickle

from skeleton import Skeleton

from math import *

from stereo_utils import camera
import pylab
import numpy

stereo_cam = camera.Camera((389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95))

def render(filename, color, theta, dest):
  skel = Skeleton(stereo_cam)
  skel.load(filename, load_PR = False)
  skel.optimize()
  skel.optimize()

  skel.plot(color, False, theta)

  xlim = pylab.xlim()
  ylim = pylab.ylim()
  xrange = xlim[1] - xlim[0]
  yrange = ylim[1] - ylim[0]
  r = max(xrange, yrange) * 0.5 # 0.32
  mid = sum(xlim) / 2
  pylab.xlim(mid - r, mid + r)
  mid = sum(ylim) / 2
  pylab.ylim(mid - r, mid + r)

if len(sys.argv) > 1:
  render(sys.argv[1], 'blue', 0.0, "foo.eps")
  pylab.legend()
  pylab.savefig("foo.eps")
  pylab.show()
  sys.exit(0)

render(sys.argv[1], 'blue', 0.0, "foo.eps")
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

if False:
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

if False:
  fig = pylab.figure(figsize=(10,10), linewidth = 0.0)

  im = Image.open("versailles.png").convert("L")
  mpp = 20. / (489-387) # meters per pixel
  imx = im.size[0] * mpp
  imy = im.size[1] * mpp

  pylab.imshow(numpy.fromstring(im.tostring(), numpy.uint8).reshape(im.size[1],im.size[0]), cmap=pylab.cm.gray, extent = (0, imx, 0, imy), interpolation = 'nearest')

  s = fig.add_subplot(111)
  s.set_position( [ 0.05, 0.05, 0.90, 0.90 ] )
  #s.set_position( [ 0.0, 0.0, 1.00, 1.00 ] )
  def xform(xs, ys):
    scale = 1.12
    xs *= scale
    ys *= scale

    f = math.pi * 0.36
    xp = xs * cos(f) - ys * sin(f)
    yp = ys * cos(f) + xs * sin(f)
    return (xp + 141,yp + 99.8)

  render("versailles_star500", "#00FF00", xform, "versailles.eps")

  pylab.savefig("versailles.eps")
  pylab.show()

if True:
  fig = pylab.figure(figsize=(10,10), linewidth = 0.0)

  edges, pts, node_labels = pickle.load(open("carson/optimized.pickle"))
  # uniq_l is unique labels
  uniq_l = sorted(set(node_labels.values()))

  # labs maps labels to sets of points
  labs = dict([ (l,set([id for (id,lab) in node_labels.items() if lab == l])) for l in uniq_l])

  cols = dict(zip(uniq_l, [ 'green', 'red', 'magenta', 'cyan', 'darkorange', 'brown', 'darkolivegreen']))

  for i,l in enumerate(labs):
    ps = [ pts[idx] for idx in labs[l] ]
    pylab.scatter([x for (x,y) in ps], [y for (x,y) in ps], color = cols[l], label = l.split('/')[-3])

  for (f0,f1) in edges:
    # This expression is 1 if the nodes are consecutive
    # abs(ordered.index(f0) - ordered.index(f1))
    p0 = pts[f0]
    p1 = pts[f1]
    p0c = cols[node_labels[f0]]
    p1c = cols[node_labels[f1]]
    if p0c == p1c:
      color = p0c
    else:
      color = 'b:'
    pylab.plot((p0[0], p1[0]), (p0[1], p1[1]), color, linewidth=2)

  if 1:
    origin = (512393.091458, 4277322.654691)
    gps = []
    for l in open("/u/prdata/vslam_data/FtCarson/2007.08/2007.08.29/course3-DTED4-run1/JPL_course3-DTED4-run1_Wed_Aug_29_13.34.43_2007_traj"):
      f = l.split()
      gps.append((float(f[5]), float(f[6])))
    pylab.plot([x-origin[0] for (x,y) in gps], [y-origin[1] for (x,y) in gps])

  pylab.legend()
  pylab.show()
  
