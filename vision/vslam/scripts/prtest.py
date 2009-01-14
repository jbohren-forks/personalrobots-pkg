import rostools
rostools.update_path('vslam')

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
random.seed(0)

cam = camera.Camera((432.0, 432.0, 0.088981018518518529, 313.78210000000001, 313.78210000000001, 220.40700000000001))
vo = VisualOdometer(cam, scavenge = True, feature_detector = FeatureDetectorFast(), descriptor_scheme = DescriptorSchemeCalonder())

vt = place_recognition.load("/u/mihelich/images/holidays/holidays.tree")

f = []
for i in range(180): # range(1,146):
  print i
  L = Image.open("pool_loop/%06dL.png" % i)
  R = Image.open("pool_loop/%06dR.png" % i)
  nf = SparseStereoFrame(L, R)
  vo.setup_frame(nf)
  nf.id = len(f)
  if vt:
    vt.add(nf.lf, nf.descriptors)
  f.append(nf)

scores = vt.topN(f[0].lf, f[0].descriptors, len(f))
# print out the closest 30
for (sc, id) in sorted(zip(scores, range(len(f))))[:30]:
  print id, sc
print
