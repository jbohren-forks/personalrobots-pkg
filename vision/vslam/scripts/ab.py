import roslib
roslib.load_manifest('vslam')

import Image
from votools import TreeOptimizer3
import place_recognition
import camera
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris, from_xyz_euler
from stereo import SparseStereoFrame
from timer import Timer

import pylab, numpy
import cPickle as pickle

import math
import random
import time
import sys
random.seed(0)

cam = camera.Camera((432.0, 432.0, 0.088981018518518529, 313.78210000000001, 313.78210000000001, 220.40700000000001))

vo = VisualOdometer(cam, scavenge = False, 
                    feature_detector = FeatureDetectorFast(), 
                    descriptor_scheme = DescriptorSchemeCalonder(),
                    inlier_error_threshold = 3.0, sba = None,
                    inlier_thresh = 99999,
                    position_keypoint_thresh = 0.2, angle_keypoint_thresh = 0.15)

dir = "dump"

f = []
for i in [ int(a) for a in sys.argv[1:] ]:
  L = Image.open("%s/%06dL.png" % (dir, i))
  R = Image.open("%s/%06dR.png" % (dir, i))
  nf = SparseStereoFrame(L, R)
  vo.setup_frame(nf)
  nf.id = len(f)
  f.append(nf)

inl,pose = vo.proximity(f[0], f[1])
print "inliers=", inl, "pose", pose.xform(0,0,0)

pairs = vo.temporal_match(f[0], f[1])
vo.show_pairs(pairs, f[0], f[1])
