import roslib
roslib.update_path('visual_odometry')
import rospy

import math
import sys
import time
import Image

sys.path.append('lib')

import rosrecord

from stereo_utils import camera
from stereo_utils.reader import reader
from stereo_utils.stereo import SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris, from_xyz_euler
import cv

class dcamImage:
  def __init__(self, m):
    if hasattr(m, "byte_data"):
      ma = m.byte_data
      self.data = ma.data
    else:
      ma = m.uint8_data # MultiArray
      self.data = ma.data
    d = ma.layout.dim
    assert d[0].label == "height"
    assert d[1].label == "width"
    self.size = (d[1].size, d[0].size)
    self.mode = "L"

  def tostring(self):
    return self.data

vo = None
fd = FeatureDetectorStar(300)
ds = DescriptorSchemeCalonder()

font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1, 1)

for cam,l_image,r_image in reader(sys.argv[1]):
  if rospy.is_shutdown():
    break
  if not vo:
    vo = VisualOdometer(cam, inlier_thresh = 75, ransac_iters = 500, scavenge = 'True')
  af = SparseStereoFrame(l_image, r_image, feature_detector = fd, descriptor_scheme = ds)
  vo.handle_frame(af)

  mono = cv.CreateImage(l_image.size, cv.IPL_DEPTH_8U, 1)
  cv.SetData(mono, l_image.tostring(), l_image.size[0])
  cvim = cv.CreateImage(l_image.size, cv.IPL_DEPTH_8U, 3)
  cv.CvtColor(mono, cvim, cv.CV_GRAY2BGR)
  inl = set([b for (a,b) in vo.pe.inl])
  for i,(u,v,d) in enumerate(af.features()):
    if i in inl:
      col = cv.RGB(0,255,0)
    else:
      col = cv.RGB(255,0,0)
    cv.Circle(cvim, (u,v), 4, col, 2)
  cv.PutText(cvim, "%4d (%d inliers)" % (af.id, vo.inl), (24,24), font, cv.RGB(255,255,255))
  cv.SaveImage("%06d.png" % af.id, cvim)

  if af.id > 100:
    break

fd.summarize_timers()
ds.summarize_timers()
vo.summarize_timers()
