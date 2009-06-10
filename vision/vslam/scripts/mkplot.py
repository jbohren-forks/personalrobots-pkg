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

import sys
import random
import unittest
import math
import copy
import pickle
import time

import vop
import Image as Image
import ImageChops as ImageChops
import ImageDraw as ImageDraw
import ImageFilter as ImageFilter

from stereo_utils.stereo import DenseStereoFrame, SparseStereoFrame
from stereo_utils.descriptor_schemes import DescriptorSchemeCalonder, DescriptorSchemeSAD
from stereo_utils.feature_detectors import FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
from visual_odometry.visualodometer import VisualOdometer, Pose
from skeleton import Skeleton
from stereo_utils.reader import reader

from math import *

import numpy
import numpy.linalg
import pylab
import cv

import rosrecord
from tf import transformations

def planar(x, y, z):
  from scipy import optimize

  def silly(sol, args):
      a,b,c = sol
      x,y,z = args
      return sum((y - (a*x + b*z + c)) ** 2)

  sol = [1.0, 1.0, 1.0]
  sol = optimize.fmin(silly, sol, args=((x,y,z),))

  a,b,c = sol
  return (a, b, c)
  return sqrt(sum((y - (a*x + b*z + c)) ** 2) / len(x))

random.seed(8)
vos = None
framecounter = 0
first_pair = None
inliers = []

keys = set()

import getopt

skel_load_filename = None
optlist, args = getopt.getopt(sys.argv[1:], "l:f:")
print optlist, args
for o,a in optlist:
  if o == '-l':
    skel_load_filename = a
  if o == '-f':
    skel_fill_filename = a

def playlist(args):
  for f in args:
    r = reader(f)
    for d in r:
      yield d + (f,)

log_dead_reckon = []

cold = True
inl_history = [0,0]
for cam,l_image,r_image,label in playlist(args):
  if vos:
    print framecounter, len(skel.nodes), "skel nodes"
  if vos == None:
    #fd = FeatureDetectorFast(300)
    fd = FeatureDetectorStar(300)
    ds = DescriptorSchemeCalonder()
    vos = [
      VisualOdometer(cam, scavenge = False,
                          inlier_error_threshold = 3.0, sba = None,
                          inlier_thresh = 100,
                          ransac_iters = 500,
                          position_keypoint_thresh = 0.2, angle_keypoint_thresh = 0.15)
    ]
    vo_x = [ [] for i in vos]
    vo_y = [ [] for i in vos]
    vo_u = [ [] for i in vos]
    vo_v = [ [] for i in vos]
    trajectory = [ [] for i in vos]
    skel = Skeleton(cam)
    if skel_load_filename:
      skel.load(skel_load_filename)
      vos[0].num_frames = max(skel.nodes) + 1
      framecounter = max(skel.nodes) + 1
    # skel.fill("/u/prdata/vslam_data/FtCarson/2007.08/2007.08.29/course3-DTED4-run1/traj.txt", 118016)
    # skel.fill("/u/prdata/vslam_data/FtCarson/2007.08/2007.08.29/course3-DTED5-run1/traj.txt", 13916)
    if skel_fill_filename:
      skel.fill(skel_fill_filename)
    skel.node_vdist = 15
    oe_x = []
    oe_y = []
    oe_home = None
  for i,vo in enumerate(vos):
    af = SparseStereoFrame(l_image, r_image, feature_detector = fd, descriptor_scheme = ds)
    vopose = vo.handle_frame(af)
    log_dead_reckon.append(vopose)
    inl_history.append(vo.inl)
    inl_history = inl_history[-2:]
    if cold:
      af.connected = -1
    else:
      af.connected = vo.inl > 7
    # Log keyframes into "pool_loop"
    # Image.fromstring("L", af.lf.size, af.lf.tostring()).save("dump/%06dL.png" % af.id)
    if False and not vo.keyframe.id in keys:
      k = vo.keyframe
      Image.fromstring("L", (640,480), k.lf.tostring()).save("dump/%06dL.png" % len(keys))
      Image.fromstring("L", (640,480), k.rf.tostring()).save("dump/%06dR.png" % len(keys))
      print "saving frame", "id", k.id, "as key", len(keys), "inliers:", k.inl, "keypoints:", len(k.kp2d), len(k.kp)
      #vo.report_frame(k)
      keys.add(k.id)

    if max(inl_history) > 7:
      skel.setlabel(label)
      novel = skel.add(vo.keyframe, vo.keyframe.connected)
    x,y,z = vo.pose.xform(0,0,0)
    trajectory[i].append((x,y,z))
    vo_x[i].append(x)
    vo_y[i].append(z)
    x1,y1,z1 = vo.pose.xform(0,0,1)
    vo_u[i].append(x1 - x)
    vo_v[i].append(z1 - z)

  print framecounter, "kp", len(af.features()), "inliers:", vo.inl
  inliers.append(vo.inl)

  def rod(p):
    m = cv.CreateMat(3, 3, cv.CV_32FC1)
    for i in range(3):
      for j in range(3):
        m[i,j] = p.M[i,j]
    r = cv.CreateMat(3, 1, cv.CV_32FC1)
    cv.Rodrigues2(m, r)
    return [ r[0,0], r[1,0], r[2,0] ]

  framecounter += 1
  LIMIT = 51248
  LIMIT = 999999
  if framecounter > LIMIT:
    break
  cold = False

  def ground_truth(p, q):
    return Pose(transformations.rotation_matrix_from_quaternion([q.x, q.y, q.z, q.w])[:3,:3], [p.x, p.y, p.z])

  gtp = None

print "There are", len(vo.tracks), "tracks"
print "There are", len([t for t in vo.tracks if t.alive]), "live tracks"
print "There are", len(set([t.p[-1] for t in vo.tracks if t.alive])), "unique endpoints on live tracks"

f = open('dead_reckon.poses', 'w')
for i,p in enumerate(log_dead_reckon):
  print >>f, i, p.tolist()
f.close()
f = open('optimized.poses', 'w')
for i in sorted(skel.nodes):
  print >>f, i, skel.newpose(i).tolist()
f.close()

fd.summarize_timers()
ds.summarize_timers()
for vo in vos:
  print vo.name()
  print "distance from start:", vo.pose.distance()
  vo.summarize_timers()
  print vo.log_keyframes
  print
skel.summarize_timers()
skel.dump_timers('skel_timers.pickle')

#skel.trim()

print "Saving as mkplot_snap"
skel.save("mkplot_snap")

skel.optimize()
skel.optimize()

skel.plot('blue', True)
pylab.savefig("mkplot.eps")
pylab.show()

sys.exit(0)

colors = [ 'red', 'black', 'magenta', 'cyan', 'orange', 'brown', 'purple', 'olive', 'gray' ]
for i in range(len(vos)):
  vos[i].planarity = planar(numpy.array([x for (x,y,z) in trajectory[i]]), numpy.array([y for (x,y,z) in trajectory[i]]), numpy.array([z for (x,y,z) in trajectory[i]]))
  print "planarity", vos[i].planarity
  xs = numpy.array(vo_x[i])
  ys = numpy.array(vo_y[i])
  if 0:
    xs -= 4.5 * 1e-3
    f = -0.06
  else:
    f = 0.0
  xp = xs * cos(f) - ys * sin(f)
  yp = ys * cos(f) + xs * sin(f)
  pylab.plot(xp, yp, c = colors[i], label = vos[i].name())
  #pylab.quiver(xp, yp, vo_u[i], vo_v[i], color = colors[i]) #, label = '_nolegend_')

  #xk = [ x for j,x in enumerate(vo_x[i]) if j in vos[i].log_keyframes ]
  #yk = [ y for j,y in enumerate(vo_y[i]) if j in vos[i].log_keyframes ]
  #pylab.scatter(xk, yk, c = colors[i], label = '_nolegend_')

pylab.plot(oe_x, oe_y, c = 'green', label = ground_truth_label)

pts = dict([ (f,skel.newpose(f.id).xform(0,0,0)) for f in skel.nodes ])
nodepts = pts.values()
pval = planar(numpy.array([x for (x,y,z) in nodepts]), numpy.array([y for (x,y,z) in nodepts]), numpy.array([z for (x,y,z) in nodepts]))
print "planarity of skeleton: ", pval

skel.optimize()

pts = dict([ (f,skel.newpose(f).xform(0,0,0)) for f in skel.nodes ])
nodepts = pts.values()
pval = planar(numpy.array([x for (x,y,z) in nodepts]), numpy.array([y for (x,y,z) in nodepts]), numpy.array([z for (x,y,z) in nodepts]))
print "planarity of skeleton: ", pval

skel.plot('blue')
print vos[0].log_keyframes


def square_aspect():
  xlim = pylab.xlim()
  ylim = pylab.ylim()
  xrange = xlim[1] - xlim[0]
  yrange = ylim[1] - ylim[0]
  r = max(xrange, yrange) * 0.75
  mid = sum(xlim) / 2
  pylab.xlim(mid - r, mid + r)
  mid = sum(ylim) / 2
  pylab.ylim(mid - r, mid + r)
pylab.legend()
pylab.savefig("foo.png", dpi=200)
#pylab.show()
