import roslib
roslib.load_manifest('stereo_utils')
import rostest
import rospy

from stereo_utils.stereo import ComputedDenseStereoFrame, SparseStereoFrame
from stereo_utils.feature_detectors import FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
from stereo_utils.descriptor_schemes import DescriptorSchemeCalonder, DescriptorSchemeSAD

import Image
import ImageDraw
import ImageChops
import random
import unittest
import math

def circle(im, x, y, r, color):
    draw = ImageDraw.Draw(im)
    box = [ int(i) for i in [ x - r, y - r, x + r, y + r ]]
    draw.pieslice(box, 0, 360, fill = color)

class TestDirected(unittest.TestCase):

  def setUp(self):
    self.img640x480 = Image.open("img1.pgm").resize((640, 480), Image.ANTIALIAS).convert("L")

  def test_sad(self):
    im = self.img640x480
    fd = FeatureDetectorStar(300)
    ds = DescriptorSchemeSAD()
    af = SparseStereoFrame(im, im, feature_detector = fd, descriptor_scheme = ds)
    for (a,b) in af.match(af):
      self.assert_(a == b)

  def test_sparse_stereo(self):
    left = Image.new("L", (640,480))
    circle(left, 320, 200, 4, 255)

    fd = FeatureDetectorStar(300)
    ds = DescriptorSchemeSAD()
    for disparity in range(20):
      right = Image.new("L", (640,480))
      circle(right, 320 - disparity, 200, 4, 255)
      sf = SparseStereoFrame(left, right, feature_detector = fd, descriptor_scheme = ds)
      self.assertAlmostEqual(sf.lookup_disparity(320,200), disparity, 0)

  def test_feature_detectors(self):
    L = self.img640x480
    R = ImageChops.offset(L, -3, 0)
    for fdt in [FeatureDetectorFast, FeatureDetectorStar, FeatureDetectorHarris]:
      feat = []
      for count in range(30,300,5):
        fd = fdt(count)
        frame = SparseStereoFrame(L, R, feature_detector = fd, descriptor_scheme = None)
        prev_feat, feat = feat, frame.features()
        # At least half the number of features requested
        self.assert_((count / 2) < len(feat))
        # Should never return too many features - too few is OK because of stereo
        self.assert_(len(feat) <= count)
        # Should have same or more features with greater count
        self.assert_(len(prev_feat) <= len(feat))
        # Previous feature call is always a sublist of this feature call
        self.assert_(feat[:len(prev_feat)] == prev_feat)

  def test_stereo_accuracy(self):
    fd = FeatureDetectorStar(300)
    ds = DescriptorSchemeSAD()
    for offset in [ 1, 10, 10.25, 10.5, 10.75, 11, 63]:
      lf = self.img640x480
      rf = self.img640x480
      rf = rf.resize((16 * 640, 480))
      rf = ImageChops.offset(rf, -int(offset * 16), 0)
      rf = rf.resize((640,480), Image.ANTIALIAS)
      for gradient in [ False, True ]:
        af = SparseStereoFrame(lf, rf, gradient, feature_detector = fd, descriptor_scheme = ds)
        kp = [ (x,y,d) for (x,y,d) in af.features() if (x > 64) ]
        error = offset - sum([d for (x,y,d) in kp]) / len(kp)
        print error
        self.assert_(abs(error) < 0.25) 

if __name__ == '__main__':
  if 0:
    rostest.unitrun('stereo_utils', 'directed', TestDirected)
  else:
    suite = unittest.TestSuite()
    suite.addTest(TestDirected('test_feature_detectors'))
    unittest.TextTestRunner(verbosity=2).run(suite)
