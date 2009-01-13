#!/usr/bin/python

import rostools
rostools.update_path('vslam')
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

from math import *

import camera

import numpy
import numpy.linalg
import pylab

import rosrecord
import transformations

from vis import Vis

def planar(x, y, z):
  from scipy import optimize

  def silly(sol, args):
      a,b,c = sol
      x,y,z = args
      return sum((y - (a*x + b*z + c)) ** 2)

  sol = [1.0, 1.0, 1.0]
  sol = optimize.fmin(silly, sol, args=((x,y,z),))

  a,b,c = sol
  return sqrt(sum((y - (a*x + b*z + c)) ** 2) / len(x))

class dcamImage:
  def __init__(self, m):
    if hasattr(m, "byte_data"):
      ma = m.byte_data
      self.data = ma.data
    else:
      ma = m.uint8_data # MultiArray
      self.data = "".join([chr(x) for x in ma.data])
    d = ma.layout.dim
    assert d[0].label == "height"
    assert d[1].label == "width"
    self.size = (d[1].size, d[0].size)
    self.mode = "L"

  def tostring(self):
    return self.data

cam = None
filename = sys.argv[1]
framecounter = 0
first_pair = None
inliers = []

if 1:
  start,end = 20000,21300
  skipto = 12634364243
  start,end = 0,3300
else:
  skipto = None
  start,end = 0,5000

print "starting loop"
#f = open(filename)
f = filename
keys = set()
for topic, msg, t in rosrecord.logplayer(f):
  if skipto and (f.tell() < skipto):
    f.seek(skipto)
  #print f.tell(), msg
  if rospy.is_shutdown():
    break

  if topic.endswith("stereo/raw_stereo") or topic.endswith("dcam/raw_stereo"):
    if not cam:
      cam = camera.StereoCamera(msg.right_info)
      print cam.params
      vos = [
        VisualOdometer(cam, scavenge = True, feature_detector = FeatureDetectorFast(), inlier_error_threshold = 3.0, sba = None,
                            inlier_thresh = 100,
                            position_keypoint_thresh = 0.2, angle_keypoint_thresh = 0.15)
      ]
      vo_x = [ [] for i in vos]
      vo_y = [ [] for i in vos]
      vo_u = [ [] for i in vos]
      vo_v = [ [] for i in vos]
      trajectory = [ [] for i in vos]
      skel = Skeleton(cam)
      oe_x = []
      oe_y = []
      oe_home = None
    if framecounter == end:
      break
    if start <= framecounter and (framecounter % 1) == 0:
      for i,vo in enumerate(vos):
        af = SparseStereoFrame(dcamImage(msg.left_image), dcamImage(msg.right_image))
        vo.handle_frame(af)
        # Log keyframes into "pool_loop"
        if not vo.keyframe.id in keys:
          k = vo.keyframe
          Image.fromstring("L", (640,480), k.lf.tostring()).save("pool_loop/%06dL.png" % len(keys))
          Image.fromstring("L", (640,480), k.rf.tostring()).save("pool_loop/%06dR.png" % len(keys))
          keys.add(k.id)

        if i == 0:
          skel.add(vo.keyframe)
          vo.correct(skel.correct_frame_pose, af)
        x,y,z = vo.pose.xform(0,0,0)
        trajectory[i].append((x,y,z))
        vo_x[i].append(x)
        vo_y[i].append(z)
        x1,y1,z1 = vo.pose.xform(0,0,1)
        vo_u[i].append(x1 - x)
        vo_v[i].append(z1 - z)
      print framecounter, vo.inl, "inliers"
      inliers.append(vo.inl)

      if 0:
        disk = {}
        for i in range(len(vos)):
          xs = numpy.array(vo_x[i])
          ys = numpy.array(vo_y[i])
          if 0:
            xs -= 4.5 * 1e-3
            fa = -0.06
          else:
            fa = 0.0
          xp = xs * cos(fa) - ys * sin(fa)
          yp = ys * cos(fa) + xs * sin(fa)
          disk['xp'] = xp
          disk['yp'] = yp

        disk['oe_x'] = oe_x
        disk['oe_y'] = oe_y

        disk['graph'] = skel.drawable()

        pf = open("foo%06d.pickle" % framecounter, 'w')
        pickle.dump(disk, pf)
        pf.close()
        if 1:
          Image.fromstring("L", (640,480), af.lf.tostring()).save("img%06d.png" % framecounter)

    if False:
      fig = pylab.figure(figsize=(10,10))
      colors = [ 'red', 'black', 'magenta', 'cyan', 'orange', 'brown', 'purple', 'olive', 'gray' ]
      for i in range(len(vos)):
        xs = numpy.array(vo_x[i])
        ys = numpy.array(vo_y[i])
        if 0:
          xs -= 4.5 * 1e-3
          fa = -0.06
        else:
          fa = 0.0
        xp = xs * cos(fa) - ys * sin(fa)
        yp = ys * cos(fa) + xs * sin(fa)
        pylab.plot(xp, yp, c = colors[i], label = vos[i].name())

      pylab.plot(oe_x, oe_y, c = 'green', label = 'ground truth')

      skel.plot('blue')

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
      pylab.savefig("foo%06d.png" % framecounter, dpi=100)
      pylab.close(fig)

    framecounter += 1

  def ground_truth(p, q):
    return Pose(transformations.rotation_matrix_from_quaternion([q.x, q.y, q.z, q.w])[:3,:3], [p.x, p.y, p.z])

  gtp = None
  if topic.endswith("odom_estimation"):
    gtp = ground_truth(msg.pose.position, msg.pose.orientation)
    ground_truth_label = "PhaseSpace"
  if topic.endswith("phase_space_snapshot"):
    gtp = ground_truth(msg.bodies[0].pose.translation, msg.bodies[0].pose.rotation)
    ground_truth_label = "wheel + IMU odometry"
  if gtp and cam:
    oe_pose = gtp
    if not oe_home:
      oe_home = oe_pose
    local = ~oe_home * oe_pose
    (x,y,z) = local.xform(0,0,0)
    oe_x.append(-y)
    oe_y.append(x)

print "There are", len(vo.tracks), "tracks"
print "There are", len([t for t in vo.tracks if t.alive]), "live tracks"
print "There are", len(set([t.p[-1] for t in vo.tracks if t.alive])), "unique endpoints on live tracks"

for vo in vos:
  print vo.name()
  print "distance from start:", vo.pose.distance()
  vo.summarize_timers()
  print vo.log_keyframes
  print
skel.summarize_timers()

pylab.figure(figsize=(10,10))
colors = [ 'red', 'black', 'magenta', 'cyan', 'orange', 'brown', 'purple', 'olive', 'gray' ]
for i in range(len(vos)):
  vos[i].planarity = planar(numpy.array([x for (x,y,z) in trajectory[i]]), numpy.array([y for (x,y,z) in trajectory[i]]), numpy.array([z for (x,y,z) in trajectory[i]]))
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

#skel.optimize()
skel.plot('blue')

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
pylab.show()
