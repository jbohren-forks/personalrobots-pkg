import roslib
roslib.update_path('visual_odometry')
import rospy

import math
import sys
import time
import Image

sys.path.append('lib')

import camera
from stereo import SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris, from_xyz_euler

Fx = 100   # Focal length x
Fy = 100   # Focal length y
Tx = 0.090 # baseline in M
Clx = 320  # Center left image
Crx = 320  # Center right image
Cy = 240   # Center Y (both images)

# Camera
cam = camera.Camera((Fx, Fy, Tx, Clx, Crx, Cy))

# Feature Detector
fd = FeatureDetectorFast(300)

# Descriptor Scheme
ds = DescriptorSchemeSAD()

# Visual Odometer
vo = VisualOdometer(cam)

for i in range(100):
  # Left image
  lim = Image.open("%s/%06dL.png" % (sys.argv[1], i))

  # Right image
  rim = Image.open("%s/%06dR.png" % (sys.argv[1], i))

  # Combine the images to make a stereo frame
  frame = SparseStereoFrame(lim, rim, feature_detector = fd, descriptor_scheme = ds)

  # Supply the stereo frame to the visual odometer
  pose = vo.handle_frame(frame)
  print i, pose
  print

vo.summarize_timers()
