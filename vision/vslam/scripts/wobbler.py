import roslib
roslib.load_manifest('vslam')
import rostest
import rospy
import place_recognition

import Image
import time
from stereo_utils import timer
import os

from raytracer import object, sphere, isphere, render_stereo_scene, ray_camera, vec3, shadeLitCloud
from math import *

import numpy
import pylab

from stereo_utils import camera
import random
import math
import sys
#import visualize
from skeleton import Skeleton

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

from stereo_utils.stereo import ComputedDenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris, from_xyz_euler

#########################################################################

#########################################################################


#VisualOdometer(stereo_cam, feature_detector = FeatureDetectorStar(), descriptor_scheme = DescriptorSchemeCalonder())
#VisualOdometer(stereo_cam, scavenge = False, sba=(1,1,10), inlier_error_threshold = 3.0),

#VisualOdometer(stereo_cam, sba=(1,1,1)),
#VisualOdometer(stereo_cam, scavenge = True, sba=None, inlier_error_threshold = 3.0),
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

def getImage(i):
  theta = (i / 400.0) * math.pi * 2
  wobble = 0.05 * (sin(theta * 9.5) + sin(theta * 3.5))
  r = 4.0 + wobble
  desired_pose = Pose(rotation(theta, 0,1,0), (4.0 - r * math.cos(theta),0,r * math.sin(theta)))
  if not os.access("/tmp/out%06d.png" % i, os.R_OK):
    cam = ray_camera(desired_pose, stereo_cam)

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
    im = Image.open("/tmp/out%06d.png" % i)
    imL,imR,_ = im.split()
  return (desired_pose, imL, imR)

fd = FeatureDetectorFast(300)
ds = DescriptorSchemeCalonder()
skel = Skeleton(stereo_cam, descriptor_scheme = ds)
vo = VisualOdometer(stereo_cam, scavenge = False, sba=None, inlier_error_threshold = 3.0)
vo.num_frames = 0

ground_truth = []
started = time.time()

connected = False
for i in range(0,800):
  print i
  (desired_pose, imL, imR) = getImage(i)
  ground_truth.append(desired_pose.xform(0,0,0))
  f = SparseStereoFrame(imL, imR, feature_detector = fd, descriptor_scheme = ds)
  vo.handle_frame(f)
  skel.add(vo.keyframe, connected)
  connected = True

if 0:
  (desired_pose, imL, imR) = getImage(408)
  ground_truth.append(desired_pose.xform(0,0,0))
  qf = SparseStereoFrame(imL, imR)
  skel.localization(qf)
  sys.exit(0)

print "nodes", skel.nodes

ended = time.time()
took = ended - started
print "Took", took, "so", 1000*took / i, "ms/frame"


skel.optimize()

#skel.save("saved-skel400")

#skel = Skeleton(stereo_cam)
#skel.load("saved-skel")
#print "about to optimize..."
#skel.optimize()

def plot_poses(Ps, c, l):
  traj = [ p.xform(0,0,0) for p in Ps ]
  fwds = [ p.xform(0,0,1) for p in Ps ]
  dirs = [ (x1-x0, y1-y0, z1-z0) for (x0,y0,z0),(x1,y1,z1) in zip(traj, fwds) ]
  if 0:
    pylab.quiver([x for (x,y,z) in traj], [z for (x,y,z) in traj], [x for (x,y,z) in dirs], [z for (x,y,z) in dirs], color = c, units = 'width', width = 1e-3)
    pylab.scatter([x for (x,y,z) in traj], [z for (x,y,z) in traj], color = c, label = l)
  else:
    pylab.scatter([x for (x,y,z) in traj], [z for (x,y,z) in traj], color = c)
    pylab.plot([x for (x,y,z) in traj], [z for (x,y,z) in traj], color = c, label = l)

sgd = []
for id in sorted([ id for id in skel.nodes ]):
  xyz,euler = skel.pg.vertex(id)
  newpose = from_xyz_euler(xyz, euler)
  sgd.append(newpose)

print vo.name()
print "distance from start:", vo.pose.distance()
vo.summarize_timers()
print vo.log_keyframes
print

skel.summarize_timers()

pylab.plot([x for (x,y,z) in ground_truth], [z for (x,y,z) in ground_truth], c = 'green', label = 'ground truth')
plot_poses(vo.log_keyposes, 'red', 'from VO')
plot_poses(sgd, 'blue', 'after SGD')
skel.plot('blue', True)
pylab.legend()
pylab.show()
#pylab.savefig("foo.png", dpi=200)
