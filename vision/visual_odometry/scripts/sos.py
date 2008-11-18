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
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, DescriptorSchemeEverything, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
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

def imgAdapted(im):
    return Image.fromstring({'mono8' : 'L', 'rgb24' : 'RGB'}[im.colorspace], (im.width, im.height), im.data).convert("L")

filename = sys.argv[1]

cam = None
prev_frame = None
framecounter = 0
sos = numpy.array([.0, .0, .0])

for topic, msg in rosrecord.logplayer(filename):
  if rospy.is_shutdown():
    break

  if topic == "videre/cal_params" and not cam:
    cam = camera.VidereCamera(msg.data)

    vo = VisualOdometer(cam, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeSAD())

  if cam and topic == "videre/images":
    imgR = imgAdapted(msg.images[0])
    imgL = imgAdapted(msg.images[1])
    assert msg.images[0].label == "right_rectified"
    assert msg.images[1].label == "left_rectified"

    frame = SparseStereoFrame(imgL, imgR)
    vo.find_keypoints(frame)
    vo.find_disparities(frame)
    vo.collect_descriptors(frame)
    if prev_frame:
      pairs = vo.temporal_match(prev_frame, frame)
      solution = vo.solve(prev_frame.kp, frame.kp, pairs, True)
      (inl, rot, shift) = solution
      sos += numpy.array(shift)
      print sos
    prev_frame = frame

    framecounter += 1
