import roslib
roslib.update_path('visual_odometry')
import rospy

import math
import sys
import time
import Image

sys.path.append('lib')

from stereo_utils import camera
from stereo_utils.stereo import SparseStereoFrame
from stereo_utils.descriptor_schemes import DescriptorSchemeCalonder, DescriptorSchemeSAD
from stereo_utils.feature_detectors import FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
from visual_odometry.pe import PoseEstimator

Fx = 389.0
Fy =  389.0
Tx = 0.08923
Clx = 323.42
Crx = 323.42
Cy = 274.95

# Camera
cam = camera.Camera((Fx, Fy, Tx, Clx, Crx, Cy))

# Feature Detector
fd = FeatureDetectorFast(300)

# Descriptor Scheme
ds = DescriptorSchemeCalonder()

# f0 is a stereo pair
f0 = SparseStereoFrame(Image.open("f0-left.png"), Image.open("f0-right.png"), feature_detector = fd, descriptor_scheme = ds)

# f1 is also a stereo pair

f1 = SparseStereoFrame(Image.open("f1-left.png"), Image.open("f1-right.png"), feature_detector = fd, descriptor_scheme = ds)

# Create a pose estimator, and set it to do 500 RANSAC iterations
pe = PoseEstimator()
pe.setNumRansacIterations(500)

# Find the correspondences between keypoints in f0 and f1.  Discard the strength component.
pairs = [ (a,b) for (a,b,_) in f1.match(f0) ]

inliers,rotation,translation = pe.estimateC(cam, f0.features(), cam, f1.features(), pairs)

print
print "inliers", inliers
print "rotation", rotation
print "translation", translation
