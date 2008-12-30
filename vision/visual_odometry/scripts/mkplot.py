#!/usr/bin/python

import rostools
rostools.update_path('visual_odometry')
import rostest
import rospy

import vop

import sys
sys.path.append('lib')
#
import visual_odometry as VO
import Image as Image
import ImageChops as ImageChops
import ImageDraw as ImageDraw
import ImageFilter as ImageFilter

import random
import unittest
import math
import copy

from image_msgs.msg import RawStereo
from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
import fast
from math import *

import camera

import numpy
import numpy.linalg
import pylab

import cairo
import array

import rosrecord
#import visualize

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

#def imgAdapted(im):
#    return Image.fromstring({'mono8' : 'L', 'rgb24' : 'RGB'}[im.colorspace], (im.width, im.height), im.data).convert("L")

class imgAdapted:
  def __init__(self, im):
    assert im.colorspace == 'mono8'
    self.size = (im.width, im.height)
    self.data = im.data
  def tostring(self):
    return self.data

class dcamImage:
  def __init__(self, m):
    ma = m.byte_data # MultiArray

    self.data = ma.data
    d = ma.layout.dim
    assert d[0].label == "height"
    assert d[1].label == "width"
    self.size = (d[1].size, d[0].size)

  def tostring(self):
    return self.data

cam = None
filename = "/u/prdata/videre-bags/loop2-color.bag"
filename = "/u/prdata/videre-bags/greenroom-2008-11-3-color.bag"
filename = "2008-11-04-09-55-12-topic.bag"
filename = "2008-11-04-14-44-56-topic.bag"
filename = "2008-11-05-14-35-11-topic.bag"
filename = "/u/prdata/videre-bags/vo1.bag"
filename = sys.argv[1]
framecounter = 0
oe_x = []
oe_y = []
first_pair = None
inliers = []


vis = Vis(filename)
for topic, msg, t in rosrecord.logplayer(filename):
  if rospy.is_shutdown():
    break

  if topic.endswith("videre/cal_params") and not cam:
    print msg.data
    cam = camera.VidereCamera(msg.data)
    (Fx, Fy, Tx, Clx, Crx, Cy) = cam.params
    Tx /= (7.30 / 7.12)
    cam = camera.Camera((Fx, Fy, Tx, Clx, Crx, Cy))

    vos = [
      VisualOdometer(cam, scavenge = True, feature_detector = FeatureDetectorFast(), inlier_error_threshold = 3.0, sba = None),

      #VisualOdometer(cam, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeSAD(), sba = (3,8,10)),

      #VisualOdometer(cam, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeSAD()),
      #VisualOdometer(cam, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeSAD(), scavenge = True),
      #VisualOdometer(cam, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeSAD(), scavenge = True, inlier_thresh = 100),

      #VisualOdometer(cam, feature_detector = FeatureDetectorHarris(), descriptor_scheme = DescriptorSchemeSAD()),
      #VisualOdometer(cam, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeSAD()),
      #VisualOdometer(cam, feature_detector = FeatureDetector4x4(FeatureDetectorFast), descriptor_scheme = DescriptorSchemeSAD()),
    ]
    vo_x = [ [] for i in vos]
    vo_y = [ [] for i in vos]
    vo_u = [ [] for i in vos]
    vo_v = [ [] for i in vos]
    trajectory = [ [] for i in vos]

  start,end = 941,1000
  start,end = 0,100

  if cam and topic.endswith("videre/images"):
    if framecounter == end:
      break
    if start <= framecounter and (framecounter % 1) == 0:
      imgR = imgAdapted(msg.images[0])
      imgL = imgAdapted(msg.images[1])
      # jdc debugging
      # Image.fromstring("L", imgL.size, imgL.tostring()).save("/tmp/mkplot-left.png")
      if not first_pair:
        first_pair = (imgL, imgR)

      for i,vo in enumerate(vos):
        af = SparseStereoFrame(imgL, imgR)
        vo.handle_frame(af)
        x,y,z = vo.pose.xform(0,0,0)
        trajectory[i].append((x,y,z))
        vo_x[i].append(x)
        vo_y[i].append(z)
        x1,y1,z1 = vo.pose.xform(0,0,1)
        vo_u[i].append(x1 - x)
        vo_v[i].append(z1 - z)
      print framecounter
    framecounter += 1

  if topic == "/dcam/raw_stereo":
    if not cam:
      cam = camera.StereoCamera(msg.right_info)
      vos = [VisualOdometer(cam, scavenge = True, feature_detector = FeatureDetectorFast(), inlier_error_threshold = 3.0, sba = None),
            ]
      vo_x = [ [] for i in vos]
      vo_y = [ [] for i in vos]
      vo_u = [ [] for i in vos]
      vo_v = [ [] for i in vos]
      trajectory = [ [] for i in vos]
    if framecounter == end:
      break
    if start <= framecounter and (framecounter % 1) == 0:
      for i,vo in enumerate(vos):
        vis.show(msg.left_image.data, [])
        af = SparseStereoFrame(dcamImage(msg.left_image), dcamImage(msg.right_image))
        vo.handle_frame(af)
        x,y,z = vo.pose.xform(0,0,0)
        trajectory[i].append((x,y,z))
        vo_x[i].append(x)
        vo_y[i].append(z)
        x1,y1,z1 = vo.pose.xform(0,0,1)
        vo_u[i].append(x1 - x)
        vo_v[i].append(z1 - z)
      print framecounter, vo.inl, "inliers"
      inliers.append(vo.inl)
    framecounter += 1

  if topic.endswith("odom_estimation"):
    oe_x.append(-msg.pose.position.y)
    oe_y.append(msg.pose.position.x)

print "There are", len(vo.tracks), "tracks"
print "There are", len([t for t in vo.tracks if t.alive]), "live tracks"
print "There are", len(set([t.p[-1] for t in vo.tracks if t.alive])), "unique endpoints on live tracks"

quality_pose = Pose()
if 0:
  # Attempt to compute best possible end-to-end pose
  vo = VisualOdometer(cam, feature_detector = FeatureDetector4x4(FeatureDetectorHarris), scavenge = True)
  f0 = SparseStereoFrame(*first_pair)
  f1 = SparseStereoFrame(imgL, imgR)
  vo.handle_frame(f0)
  vo.handle_frame(f1)
  quality_pose = vo.pose

if 0:
  for t in vos[2].all_tracks:
    pylab.plot([x for (x,y,d) in t.p], [y for (x,y,d) in t.p])
  pylab.xlim((0, 640))
  pylab.ylim((0, 480))
  pylab.savefig("foo.png", dpi=200)
  pylab.show()
  sys.exit(0)

pylab.figure(figsize=(20,20))
colors = [ 'blue', 'red', 'black', 'magenta', 'cyan', 'orange', 'brown', 'purple', 'olive', 'gray' ]
for i in range(len(vos)):
  vos[i].planarity = planar(numpy.array([x for (x,y,z) in trajectory[i]]), numpy.array([y for (x,y,z) in trajectory[i]]), numpy.array([z for (x,y,z) in trajectory[i]]))
  xs = numpy.array(vo_x[i])
  ys = numpy.array(vo_y[i])
  xs -= 4.5 * 1e-3
  f = -0.06
  xp = xs * cos(f) - ys * sin(f)
  yp = ys * cos(f) + xs * sin(f)
  pylab.plot(xp, yp, c = colors[i], label = vos[i].name())
  #pylab.quiver(xp, yp, vo_u[i], vo_v[i], color = colors[i]) #, label = '_nolegend_')

  #xk = [ x for j,x in enumerate(vo_x[i]) if j in vos[i].log_keyframes ]
  #yk = [ y for j,y in enumerate(vo_y[i]) if j in vos[i].log_keyframes ]
  #pylab.scatter(xk, yk, c = colors[i], label = '_nolegend_')

pylab.plot(oe_x, oe_y, c = 'green', label = 'wheel + IMU odometry')
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

for vo in vos:
  print vo.name()
  print "distance from start:", vo.pose.distance()
  print "planarity", vo.planarity
  print "pose", vo.pose.comparison(quality_pose)
  vo.summarize_timers()
  print vo.log_keyframes
  print
