import rostools
rostools.update_path('visual_odometry')
import rostest
import rospy

import Image

from raytracer import object, sphere, isphere, render_stereo_scene, ray_camera, vec3, shadeLitCloud
from math import *

if 0:
  pylab.hist(noise(vec3(arrayrange(100000) * 1e-2, 7.7, 9.0)), bins=100)
  pylab.show()
  sys.exit(1)

import numpy
import pylab

import camera
import random
import math
import sys
#import visualize

def rotation(angle, x, y, z):
  return numpy.array([
    [ 1 + (1-cos(angle))*(x*x-1)         ,    -z*sin(angle)+(1-cos(angle))*x*y   ,    y*sin(angle)+(1-cos(angle))*x*z  ],
    [ z*sin(angle)+(1-cos(angle))*x*y    ,    1 + (1-cos(angle))*(y*y-1)         ,    -x*sin(angle)+(1-cos(angle))*y*z ],
    [ -y*sin(angle)+(1-cos(angle))*x*z   ,    x*sin(angle)+(1-cos(angle))*y*z    ,    1 + (1-cos(angle))*(z*z-1)       ]])

stereo_cam = camera.Camera((389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95))

random.seed(0)
scene = []
blobs = []
if 0:
  for obj in range(100):
    theta = random.random() * math.pi * 2
    center = vec3(math.cos(theta), math.sin(theta), obj / 10.)
    blobs.append(center)
else:
  for obj in range(180):
    theta = (obj / 180.0) * math.pi * 2
    phi = random.random() * math.pi * 2
    rmaj = 4.0
    rmin = 1.0 + random.random()
    costh = math.cos(theta)
    sinth = math.sin(theta)
    r = rmaj + rmin * math.cos(phi)
    center = vec3(rmaj - r * costh, rmin * math.sin(phi), r * sinth)
    blobs.append(center)

#scene += [object(isphere(vec3(0,0,0), 1000), shadeLitCloud, {'scale':0.001})]

from stereo import ComputedDenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris

class imgStereo:
  def __init__(self, im):
    self.size = im.size
    self.data = im.tostring()
  def tostring(self):
    return self.data

class PhonyVisualOdometer(VisualOdometer):
  def find_keypoints(self, frame):
    pass
  def find_disparities(self, frame):
    pass
  def collect_descriptors(self, frame):
    pass
  def temporal_match(self, af0, af1, want_distances = False):
    return [(i,i) for i in range(400)]

points = [ (-1 + 2 * random.random(), -1 + 2 * random.random(), 1 + 5 * random.random()) for i in range(400)]

class PhonyFrame:
  def __init__(self, pose, cam):
    p3d = [pose.xform(*p) for p in points]
    self.kp = [cam.cam2pix(*p) for p in p3d]
    def r(): return -0.5 + random.random() * 1
    self.kp = [(x+r(),y+r(),d) for (x,y,d) in self.kp]

vos = [
  #VisualOdometer(stereo_cam, sba=(1,1,1)),
  #VisualOdometer(stereo_cam, scavenge = True, sba=None, inlier_error_threshold = 3.0),
  VisualOdometer(stereo_cam, scavenge = False, sba=(1,1,10), inlier_error_threshold = 3.0),
  VisualOdometer(stereo_cam, scavenge = False, sba=(1,1,10), inlier_error_threshold = 3.0),
  #VisualOdometer(stereo_cam, sba=(5,5,10), inlier_error_threshold = 1.5),
  #VisualOdometer(stereo_cam, sba=(5,5,10), inlier_error_threshold = 2.0),
  #VisualOdometer(stereo_cam, sba=(5,5,10), inlier_error_threshold = 2.5),
  #VisualOdometer(stereo_cam, sba=(5,5,10), inlier_error_threshold = 3.0),
  #VisualOdometer(stereo_cam, sba=(5,5,10), inlier_error_threshold = 3.5),
  #VisualOdometer(stereo_cam, feature_detector = FeatureDetector4x4(FeatureDetectorHarris), scavenge = True, sba = None),

  #PhonyVisualOdometer(stereo_cam),
  #PhonyVisualOdometer(stereo_cam, sba = (5,5,5))
  #VisualOdometer(stereo_cam, feature_detector = FeatureDetector4x4(FeatureDetectorFast)),
  #VisualOdometer(stereo_cam, scavenge = True, sba = (1,1,5)),
]

class MagicStereoFrame(SparseStereoFrame):
  def __init__(self, L, R, D, cam):
    SparseStereoFrame.__init__(self, L, R)
    self.D = D
    self.cam = cam
  def lookup_disparity(self, px, py):
    x = self.D[0].getpixel((px,py))
    y = self.D[1].getpixel((px,py))
    z = self.D[2].getpixel((px,py))
    (x,y,d) = self.cam.cam2pix(x, y, z)
    #assert abs(x - px) < 2
    return d

evaluate_disparity = False
use_magic_disparity = False

disparities = []

trajectory = [ [] for i in vos]
for i in range(401):
  print i
  if 0:
    desired_pose = Pose(rotation(0, 0,1,0), (0,0,0.01*i))
  else:
    theta = (i / 400.0) * math.pi * 2
    r = 4.0
    desired_pose = Pose(rotation(theta, 0,1,0), (r - r * math.cos(theta),0,r * math.sin(theta)))
  cam = ray_camera(desired_pose, stereo_cam)

  if 0: # set to 1 for the first time to generate cache
    imL = Image.new("RGB", (640,480))
    imR = Image.new("RGB", (640,480))
    imD = [ Image.new("F", (640,480)), Image.new("F", (640,480)), Image.new("F", (640,480)) ]
    def is_visible(p, pt):
      camx,camy,camz = (~p).xform(pt.x, pt.y, pt.z)
      if camz <= 0:
        return False
      if abs(camx / camz) > 2.0:
        return False
      if abs(camy / camz) > 2.0:
        return False
      return True
    scene = [object(sphere(center, 0.3), shadeLitCloud, {'scale':10}) for center in blobs if is_visible(desired_pose, center)]
    scene += [object(isphere(vec3(0,0,0), 1000), shadeLitCloud, {'scale':0.001})]
    render_stereo_scene(imL, imR, imD, cam, scene, i * 0.033)
    imL = imL.convert("L")
    imR = imR.convert("L")
    im = Image.merge("RGB", (imL, imR, imR))
    im.save("/tmp/out%06d.png" % i)
    imD[0].save("/tmp/out%06d-x.tiff" % i)
    imD[1].save("/tmp/out%06d-y.tiff" % i)
    imD[2].save("/tmp/out%06d-z.tiff" % i)
  else:
    fn = i
    im = Image.open("/tmp/out%06d.png" % fn)
    imL,imR,_ = im.split()
    if use_magic_disparity:
      imD = [ Image.open("/tmp/out%06d-x.tiff" % fn), Image.open("/tmp/out%06d-y.tiff" % fn), Image.open("/tmp/out%06d-z.tiff" % fn) ]
    if 0:
      imL,imR = imR,imL
      imL = imL.transpose(Image.FLIP_LEFT_RIGHT)
      imR = imR.transpose(Image.FLIP_LEFT_RIGHT)
      for i in range(3):
        imD[i] = imD[i].transpose(Image.FLIP_LEFT_RIGHT)
  if evaluate_disparity:
    f0 = MagicStereoFrame(imgStereo(imL), imgStereo(imR), imD, stereo_cam)
    f1 = SparseStereoFrame(imgStereo(imL), imgStereo(imR))
    vos[0].find_keypoints(f0)
    pairs = [(f0.lookup_disparity(x,y), f1.lookup_disparity(x,y)) for (x,y) in f0.kp2d]
    disparities += [(a,b) for (a,b) in pairs if b != None]
  else:
    for j,vo in enumerate(vos):
      if use_magic_disparity:
        f = MagicStereoFrame(imgStereo(imL), imgStereo(imR), imD, stereo_cam)
      else:
        f = SparseStereoFrame(imgStereo(imL), imgStereo(imR))

      #f = PhonyFrame(~desired_pose, stereo_cam)
      #vo.lock_handle_frame(f)

      if j == 1:
        fext = SparseStereoFrame(imgStereo(imL), imgStereo(imR))
        #fext.pose = desired_pose * Pose(rotation(0, 0,1,0), (0.1, 0, 0))
        #fext.pose = Pose()
        fext.pose = desired_pose
        vo.add_external_frame(f, fext)
        vo.handle_frame(f)
        vo.maintain_tracks(f, fext)
      else:
        vo.handle_frame(f)
      #visualize.viz(vo, f)
      (x,y,z) = vo.pose.xform(0,0,0)
      if 0:
        print vo.name(), (x,y,z)
        (ex,ey,ez) = desired_pose.xform(0,0,0)
        Ys[j].append(z - ez)
      else:
        trajectory[j].append((x,y,z))

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

if evaluate_disparity:
  pylab.scatter([a for (a,b) in disparities], [b for (a,b) in disparities])
  pylab.xlim((0, 64))
  pylab.ylim((0, 64))
else:
  if 1:
    colors = [ 'blue', 'red', 'green', 'black', 'magenta', 'cyan' ]
    for i in range(len(vos)):
      #print trajectory[i]
      pylab.plot([x for (x,y,z) in trajectory[i]], [z for (x,y,z) in trajectory[i]], label = vos[i].name(), color = colors[i])

    xlim = pylab.xlim()
    ylim = pylab.ylim()
    xrange = xlim[1] - xlim[0]
    yrange = ylim[1] - ylim[0]
    r = max(xrange, yrange) * 1.5
    mid = sum(xlim) / 2
    pylab.xlim(mid - 0.5 * r, mid + 0.5 * r)
    mid = sum(ylim) / 2
    pylab.ylim(mid - 0.5 * r, mid + 0.5 * r)
  else:
    for t in vos[0].all_tracks:
      pylab.plot([x for (x,y,d) in t.p], [y for (x,y,d) in t.p])
    pylab.xlim((0, 640))
    pylab.ylim((0, 480))

pylab.legend()
#pylab.savefig("foo.png")
pylab.show()

quality_pose = Pose()

for i,vo in enumerate(vos):
  vos[i].planarity = planar(numpy.array([x for (x,y,z) in trajectory[i]]), numpy.array([y for (x,y,z) in trajectory[i]]), numpy.array([z for (x,y,z) in trajectory[i]]))
  print vo.name()
  print "distance from start:", vo.pose.distance()
  print "planarity", vo.planarity
  print "pose", vo.pose.comparison(quality_pose)
  vo.summarize_timers()
  print vo.log_keyframes
  print
