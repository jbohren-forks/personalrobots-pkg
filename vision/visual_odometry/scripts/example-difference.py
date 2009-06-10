import roslib
roslib.load_manifest('visual_odometry')
import rospy
import Image
import math

import os

from stereo_utils import camera
from stereo_utils.stereo import SparseStereoFrame, ComputedDenseStereoFrame
from stereo_utils.descriptor_schemes import DescriptorSchemeCalonder, DescriptorSchemeSAD
from stereo_utils.feature_detectors import FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
from visual_odometry.visualodometer import VisualOdometer, Pose
from visual_odometry.pe import PoseEstimator
import vop
import cv

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

# f0 is a stereo pair, note that this code needs dense stereo
f0 = ComputedDenseStereoFrame(Image.open("f0-left.png"), Image.open("f0-right.png"), feature_detector = fd, descriptor_scheme = ds)

# f1 is also a stereo pair

f1 = ComputedDenseStereoFrame(Image.open("f1-left.png"), Image.open("f1-right.png"), feature_detector = fd, descriptor_scheme = ds)

# Create a pose estimator, and set it to do 500 RANSAC iterations
pe = PoseEstimator()
pe.setNumRansacIterations(500)

# Find the correspondences between keypoints in f0 and f1.  Discard the strength component.
pairs = [ (a,b) for (a,b,_) in f1.match(f0) ]

# Find the pose
inliers,rotation,translation = pe.estimateC(cam, f0.features(), cam, f1.features(), pairs)

def xform(M, x, y, z):
  nx = vop.mad(M[0], x, vop.mad(M[1], y, vop.mad(M[2], z, M[3])))
  ny = vop.mad(M[4], x, vop.mad(M[5], y, vop.mad(M[6], z, M[7])))
  nz = vop.mad(M[8], x, vop.mad(M[9], y, vop.mad(M[10], z, M[11])))
  return (nx, ny, nz)

chipsize = (640,480)
factor = 640 / chipsize[0]

def img_err(f0, f1, RT, stereo_cam):
  # Each point in f1's point cloud, compute xyz
  # Transform those points from f1's frame into f0's
  # Compute the uvd of each point
  # Sample f0 image using the uv
  # Returns f0 warped into f1's frame

  def f_uvd(f):
    d_str = "".join([ chr(min(x / 4, 255)) for x in f.disp_values])
    d = vop.array(Image.fromstring("L", (640,480), d_str).resize(chipsize, Image.NEAREST).tostring())
    f_valid = d != 255.0
    f_d = d / 4
    u = vop.duplicate(vop.arange(chipsize[0]), chipsize[1]) * factor
    v = vop.replicate(vop.arange(chipsize[1]), chipsize[0]) * factor
    i = vop.array(Image.fromstring("L", (640,480), f.rawdata).resize(chipsize, Image.NEAREST).tostring()) / 255.
    return (f_valid, u, v, f_d, i)

  # Compute validity and (u,v,d) for both images
  f0p,f0u,f0v,f0d,f0i = f_uvd(f0)
  f1p,f1u,f1v,f1d,f1i = f_uvd(f1)

  # Take f1 in camera (x,y,z)
  (f1x,f1y,f1z) = stereo_cam.pix2cam(f1u, f1v, f1d)

  # Pass that via the given transform, to get transformed (x,y,z)
  (f1x0,f1y0,f1z0) = xform(RT, f1x, f1y, f1z)

  # Now take those x,y,z and map to u,v,d
  (f1u0,f1v0,f1d0) = stereo_cam.cam2pix(f1x0,f1y0,f1z0)

  # Sample f0 using those (u,v,d) points; this gives f0 in f1's frame, called f1i0

  addr = vop.floor(f1u0 / factor) + chipsize[0] * vop.floor(f1v0 / factor)
  ok = f1p & (0 <= f1u0) & (f1u0 < 640) & (0 <= f1v0) & (f1v0 < 480)
  addr = vop.where(ok, addr, 0)
  f1i0 = vop.take(f0i, addr)

  # Return:
  #  ok: which pixels are reliable in f1i0
  #  f1p: which pixels are good in f1
  #  f1i0: as computed above
  #  f1i: the original f1 image
  return (ok, f1p, f1i0, f1i)

def img_img(f0, f1, RT, stereo_cam):
  (ok, f1p, f1i0, f1i) = img_err(f0, f1, RT, stereo_cam)
  c_r = vop.where(ok, f1i, 0)
  c_g = vop.where(ok, f1i0, 0)
  return c_r, c_g

def vop2iplimage(v):
  ipl = cv.CreateImage((640,480), 8, 1)
  cv.SetData(ipl, (v * 255).tostring(), 640)
  return ipl

R = list(rotation)
diffpose = (
    R[0:3] + [translation[0]] +
    R[3:6] + [translation[1]] +
    R[6:9] + [translation[2]] +
    [ 0, 0, 0, 1 ])
a,b = img_img(f0, f1, diffpose, cam)
ai = vop2iplimage(a)
bi = vop2iplimage(b)
# a is red
# b is green/blue
def mergeRB(ai, bi):
  img = cv.CreateImage((640,480), 8, 3)
  cv.Merge(bi, bi, ai, None, img)
  return img

cv.NamedWindow("simple-difference", 1)
cv.NamedWindow("warped-difference", 1)
cv.ShowImage("simple-difference", mergeRB(cv.LoadImage("f0-left.png", 0), cv.LoadImage("f1-left.png", 0)))
cv.ShowImage("warped-difference", mergeRB(ai, bi))
cv.WaitKey()

