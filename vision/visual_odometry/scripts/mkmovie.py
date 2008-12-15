#!/usr/bin/python

import rostools
rostools.update_path('visual_odometry')
import rostest
import rospy

import vop

import sys
sys.path.append('lib')

import visual_odometry as VO
import Image as Image
import ImageChops as ImageChops
import ImageDraw as ImageDraw
import ImageFilter as ImageFilter

import random
import unittest
import math
import copy

from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
import fast
from math import *

import camera

import numpy
import numpy.linalg
#import pylab

import cairo
import array

import rosrecord
import visualize

class imgAdapted:
  def __init__(self, im):
    assert im.colorspace == 'mono8'
    self.size = (im.width, im.height)
    self.data = im.data
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
for topic, msg in rosrecord.logplayer(filename):
  if rospy.is_shutdown():
    break

  if topic.endswith("videre/cal_params") and not cam:
    print msg.data
    cam = camera.VidereCamera(msg.data)
    print "HERE"

    #vo = VisualOdometer(cam, inlier_thresh = 999999, descriptor_scheme = DescriptorSchemeCalonder())
    #vo2 = VisualOdometer(cam, inlier_thresh = 999999, descriptor_scheme = DescriptorSchemeSAD())

    #vo1 = VisualOdometer(cam, feature_detector = FeatureDetectorHarris(), descriptor_scheme = DescriptorSchemeCalonder())
    #vo2 = VisualOdometer(cam, feature_detector = FeatureDetectorHarris(), descriptor_scheme = DescriptorSchemeSAD())

    #vo = VisualOdometer(cam, descriptor_scheme = DescriptorSchemeCalonder())
    #vo2 = VisualOdometer(cam, descriptor_scheme = DescriptorSchemeSAD())

    #vo = VisualOdometer(cam, feature_detector = FeatureDetectorFast())
    #vo2 = VisualOdometer(cam, feature_detector = FeatureDetector4x4(FeatureDetectorFast))
    #vos = [vo1,vo2]
    vos = [
      VisualOdometer(cam, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeSAD(), sba = True),
#      VisualOdometer(cam, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeSAD(), scavenge = True),
    ]

  start,end = 0,2

  if cam and topic.endswith("videre/images"):
    print framecounter
    if framecounter == end:
      break
    if start <= framecounter and (framecounter % 1) == 0:
      imgR = imgAdapted(msg.images[0])
      imgL = imgAdapted(msg.images[1])

      w,h = imgL.size
      comp = Image.new("RGB", (w*2,h*2))

      if 0:
        for i,vo in enumerate(vos):
          af = SparseStereoFrame(imgL, imgR)
          vo.handle_frame(af)
          if 1:
            leftframe = visualize.render_source_with_keypoints(vo, af)
            comp.paste(leftframe, (w*i,0))
            leftframe = visualize.render_text(vo, af)
            comp.paste(leftframe, (w*i,h))
          else:
            if af.id == 0:
              master_frame = af
            else:
              print "id", af.id, "inliers:", vo.inl, "proximity:", vo.proximity(master_frame, af)
            comp = visualize.render_source_with_keypoints_stats(vo, af)
          comp.save("/tmp/im%04d.tiff" % (framecounter - start))
      else:
        vo = vos[0]
        af = SparseStereoFrame(imgL, imgR)
        vo.handle_frame(af)
        if 0:
          inliers = [ (P0,P1) for (P1,P0) in vo.pe.inliers()]
          if inliers != []:
            pts3d = [cam.pix2cam(*P1) for (P0,P1) in inliers]
            X = [ x for (x,y,z) in pts3d ]
            Y = [ y for (x,y,z) in pts3d ]
            Z = [ z for (x,y,z) in pts3d ]
            print "xrange", min(X), max(X)
            print "yrange", min(Y), max(Y)
            print "zrange", min(Z), max(Z)
        visualize.viz(vo, af)

    framecounter += 1

print "distance from start:", vo.pose.distance()
for v in vos:
  print v.feature_detector.name(), v.descriptor_scheme.__class__.__name__
  v.summarize_timers()
  print v.log_keyframes
  print
