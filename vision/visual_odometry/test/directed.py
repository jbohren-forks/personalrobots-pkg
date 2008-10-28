import rostools
rostools.update_path('visual_odometry')
import rostest

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

from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose
import fast
from math import *

import camera

import numpy
import numpy.linalg
#import pylab

import cairo
import array

def rotation(angle, x, y, z):
  return numpy.array([
    [ 1 + (1-cos(angle))*(x*x-1)         ,    -z*sin(angle)+(1-cos(angle))*x*y   ,    y*sin(angle)+(1-cos(angle))*x*z  ],
    [ z*sin(angle)+(1-cos(angle))*x*y    ,    1 + (1-cos(angle))*(y*y-1)         ,    -x*sin(angle)+(1-cos(angle))*y*z ],
    [ -y*sin(angle)+(1-cos(angle))*x*z   ,    x*sin(angle)+(1-cos(angle))*y*z    ,    1 + (1-cos(angle))*(z*z-1)       ]])

def circle(im, x, y, r, color):
    draw = ImageDraw.Draw(im)
    box = [ int(i) for i in [ x - r, y - r, x + r, y + r ]]
    draw.pieslice(box, 0, 360, fill = color)

class TestDirected(unittest.TestCase):

  def test_sad(self):
    dir = "/u/konolige/vslam/data/indoor1/"
    cam = camera.Camera((389.0, 389.0, 89.23, 323.42, 323.42, 274.95))
    vo = VisualOdometer(cam)

    class adapter:
      def __init__(self, im):
        self.rawdata = im.tostring()
        self.size = im.size

    im = adapter(Image.open("%s/left-%04d.ppm" % (dir,1000)))
    vo.feature_detector.thresh *= 15
    vo.find_keypoints(im)
    im.kp = im.kp2d
    vo.collect_descriptors(im)
    print len(im.kp)
    print vo.temporal_match(im, im)

  def xtest_smoke(self):
    cam = camera.Camera((389.0, 389.0, 89.23, 323.42, 323.42, 274.95))
    vo = VisualOdometer(cam)
    vo.reset_timers()
    dir = "/u/konolige/vslam/data/indoor1/"

    trail = []
    prev_scale = None

    schedule = [(f+1000) for f in (range(0,100) + range(100,0,-1) + [0]*10)]
    schedule = range(1507)
    for f in schedule:
      lf = Image.open("%s/left-%04d.ppm" % (dir,f))
      rf = Image.open("%s/right-%04d.ppm" % (dir,f))
      lf.load()
      rf.load()
      af = SparseStereoFrame(lf, rf)

      vo.handle_frame(af)
      print f, vo.inl
      trail.append(numpy.array(vo.pose.M[0:3,3].T)[0])
    def d(a,b):
      d = a - b
      return sqrt(numpy.dot(d,d.conj()))
    print "covered   ", sum([d(a,b) for (a,b) in zip(trail, trail[1:])])
    print "from start", d(trail[0], trail[-1]), trail[0] - trail[-1]

    vo.summarize_timers()
    print vo.log_keyframes

  def test_sparse_stereo(self):
    left = Image.new("L", (640,480))
    circle(left, 320, 200, 4, 255)

    for disparity in range(20):
      right = Image.new("L", (640,480))
      circle(right, 320 - disparity, 200, 4, 255)
      sf = SparseStereoFrame(left, right)
      self.assertAlmostEqual(sf.lookup_disparity(320,200), disparity, 0)

  def test_solve_spin(self):
    # Test process with one 'ideal' camera, one real-world Videre
    camera_param_list = [
        (200.0, 200.0, 3.00,  320.0, 320.0, 240.0),
        (389.0, 389.0, 89.23, 323.42, 323.42, 274.95),
    ]
    for cam_params in camera_param_list:
      cam = camera.Camera(cam_params)
      vo = VisualOdometer(cam)

      kps = []
      model = [ (x*200,y*200,z*200) for x in range(-3,4) for y in range(-3,4) for z in range(-3,4) ]
      for angle in range(80):
        im = Image.new("L", (640, 480))
        theta = (angle / 80.) * (pi * 2)
        R = rotation(theta, 0, 1, 0)
        kp = []
        for (mx,my,mz) in model:
          pp = None
          pt_camera = (numpy.dot(numpy.array([mx,my,mz]), R))
          (cx,cy,cz) = numpy.array(pt_camera).ravel()
          if cz > 100:
            (x,y,d) = cam.cam2pix(cx, cy, cz)
            if 0 <= x and x < 640 and 0 <= y and y < 480:
              pp = (x,y,d)
              circle(im, x, y, 2, 255)
          kp.append(pp)
        kps.append(kp)

      expected_rot = numpy.array(numpy.mat(rotation(2 * pi / 80, 0, 1, 0))).ravel()

      for i in range(100):
        i0 = i % 80
        i1 = (i + 1) % 80
        pairs = [ (i,i) for i in range(len(model)) if (kps[i0][i] and kps[i1][i]) ]
        def sanify(L, sub):
          return [i or sub for i in L]
        (inliers, rot, shift) = vo.solve(sanify(kps[i0],(0,0,0)), sanify(kps[i1],(0,0,0)), pairs)
        self.assert_(inliers != 0)
        self.assertAlmostEqual(shift[0], 0.0, 3)
        self.assertAlmostEqual(shift[1], 0.0, 3)
        self.assertAlmostEqual(shift[2], 0.0, 3)
        for (et, at) in zip(rot, expected_rot):
          self.assertAlmostEqual(et, at, 3)

  def xtest_sim(self):
    # Test process with one 'ideal' camera, one real-world Videre
    camera_param_list = [
      # (200.0, 200.0, 3.00,  320.0, 320.0, 240.0),
      (389.0, 389.0, 89.23, 323.42, 323.42, 274.95)
    ]
    def move_combo(i):
      R = rotation(i*0.02, 0, 1, 0)
      S = (i * -0.01,0,0)
      return Pose(R, S)
    def move_translate(i):
      R = rotation(0, 0, 1, 0)
      S = (0,0,0)
      return Pose(R, S)
    def move_Yrot(i):
      R = rotation(i*0.02, 0, 1, 0)
      S = (i * 0,0,0)
      return Pose(R, S)

    for movement in [ move_combo, move_Yrot ]:
      for cam_params in camera_param_list:
        cam = camera.Camera(cam_params)

        kps = []
        model = [ (x*200,y*200,z*200) for x in range(-3,4) for y in range(-3,4) for z in range(-3,4) ]
        def rndimg():
          b = "".join(random.sample([ chr(c) for c in range(256) ], 64))
          return Image.fromstring("L", (8,8), b)
        def sprite(dst, x, y, src):
          try:
            dst.paste(src, (int(x)-4,int(y)-4))
          except:
            print "paste failed", x, y
        random.seed(0)
        palette = [ rndimg() for i in model ]
        expected = []
        afs = []
        for i in range(100):
          P = movement(i)
          li = Image.new("L", (640, 480))
          ri = Image.new("L", (640, 480))
          q = 0
          for (mx,my,mz) in model:
            pp = None
            pt_camera = (numpy.dot(P.M.I, numpy.array([mx,my,mz,1]).T))
            (cx,cy,cz,cw) = numpy.array(pt_camera).ravel()
            if cz > 100:
              ((xl,yl),(xr,yr)) = cam.cam2pixLR(cx, cy, cz)
              if 0 <= xl and xl < 640 and 0 <= yl and yl < 480:
                sprite(li, xl, yl, palette[q])
                sprite(ri, xr, yr, palette[q])
            q += 1
          #li.save("sim/left-%04d.png" % i)
          #ri.save("sim/right-%04d.png" % i)
          expected.append(P)
          afs.append(SparseStereoFrame(li, ri))

      threshes = [0.0, 0.01, 0.02, 0.03, 0.04, 0.05, 0.06]
      threshes = [ 0.001 * i for i in range(100) ]
      threshes = range(100, 500, 10)
      error = []
      for thresh in threshes:
        vo = VisualOdometer(cam, inlier_thresh = thresh)
        for (e,af) in zip(expected, afs)[::1]:
          vo.handle_frame(af)
        error.append(e.compare(vo.pose)[0])
      print threshes, error
      pylab.scatter(threshes, error)
    pylab.show()

  def test_solve_rotation(self):

    cam = camera.Camera((389.0, 389.0, 89.23, 323.42, 323.42, 274.95))
    vo = VisualOdometer(cam)

    model = []

    radius = 1200.0

    kps = []
    for angle in range(80):
      im = Image.new("L", (640, 480))
      theta = (angle / 80.) * (pi * 2)
      R = rotation(theta, 0, 1, 0)
      kp = []
      for s in range(7):
        for t in range(7):
          y = -400
          pt_model = numpy.array([110 * (s - 3), y, 110 * (t - 3)]).transpose()
          pt_camera = (numpy.dot(pt_model, R) + numpy.array([0, 0, radius])).transpose()
          (cx, cy, cz) = [ float(i) for i in pt_camera ]
          (x,y,d) = cam.cam2pix(cx, cy, cz)
          kp.append((x,y,d))
          circle(im, x, y, 2, 255)
      kps.append(kp)

    expected_shift = 2 * radius * sin(pi / 80)

    for i in range(100):
      i0 = i % 80
      i1 = (i + 1) % 80
      pairs = zip(range(len(kps[i0])), range(len(kps[i1])))
      (inliers, rod, shift) = vo.solve(kps[i0], kps[i1], pairs)
      actual_shift = sqrt(shift[0]*shift[0] + shift[1]*shift[1] + shift[2]*shift[2])

      # Should be able to estimate camera shift to nearest thousandth of mm
      self.assertAlmostEqual(actual_shift, expected_shift, 3)

  def xtest_image_pan(self):
    cam = camera.Camera((1.0, 1.0, 89.23, 320., 320., 240.0))
    vo = VisualOdometer(cam)
    prev_af = None
    pose = None
    im = Image.open("img1.pgm")
    for x in [0,5]: # range(0,100,10) + list(reversed(range(0, 100, 10))):
      lf = im.crop((x, 0, x + 640, 480))
      rf = im.crop((x, 0, x + 640, 480))
      af = SparseStereoFrame(lf, rf)

      vo.find_keypoints(af)

      vo.find_disparities(af)
      vo.collect_descriptors(af)

      if prev_af:
        pairs = vo.temporal_match(prev_af, af)
        pose = vo.solve(prev_af.kp, af.kp, pairs)
        for i in range(10):
          old = prev_af.kp[pairs[i][0]]
          new = af.kp[pairs[i][1]]
          print old, new, new[0] - old[0]
      prev_af = af
      print "frame", x, "has", len(af.kp), "keypoints", pose

if __name__ == '__main__':
    rostest.unitrun('visual_odometry', 'directed', TestDirected)
    if 0:
        suite = unittest.TestSuite()
        suite.addTest(TestDirected('test_sad'))
        unittest.TextTestRunner(verbosity=2).run(suite)
