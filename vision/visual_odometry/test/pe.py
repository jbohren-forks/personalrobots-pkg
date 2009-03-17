import roslib
roslib.update_path('visual_odometry')
import rostest
import rospy

import vop

import math
import sys
import time

sys.path.append('lib')
import votools

import Image

from tf import transformations
from stereo import ComputedDenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris, from_xyz_euler

import camera
import pylab, numpy

import random
random.seed(0)

P0 = [ 
  1.0, 1.0, 1.0,
  2.0, 1.0, 1.0,
  1.0, 2.0, 1.0
]
P1 = [ 
  2.0, 2.0, 2.0,
  2.0, 1.0, 1.0,
  1.0, 2.0, 1.0
]
print votools.SVD(P0, P1)

if 0:
  started = time.time()
  for i in range(500 * 1000):
    votools.SVD(P0, P1)
  print "Took", time.time() - started

stereo_cam = camera.Camera((389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95))

vo = VisualOdometer(stereo_cam)

#(f0,f1) = [ ComputedDenseStereoFrame(Image.open("../vslam/kk2/%06dL.png" % i), Image.open("../vslam/kk2/%06dR.png" % i)) for i in [670, 671] ]
(f0,f1) = [ ComputedDenseStereoFrame(Image.open("f%d-left.png" % i), Image.open("f%d-right.png" % i)) for i in [0, 1] ]
d = "/u/jamesb/ros/ros-pkg/vision/vslam/trial"
#(f0,f1) = [ ComputedDenseStereoFrame(Image.open("%s/%06dL.png" % (d,i)), Image.open("%s/%06dR.png" % (d,i))) for i in [276, 278] ]
#(f0,f1) = [ ComputedDenseStereoFrame(Image.open("../vslam/trial/%06dL.png" % i), Image.open("../vslam/trial/%06dR.png" % i)) for i in [277, 278] ]
#(f0,f1) = [ ComputedDenseStereoFrame(Image.open("../vslam/trial/%06dL.png" % i), Image.open("../vslam/trial/%06dR.png" % i)) for i in [0, 1] ]

chipsize = (640,480)
factor = 640 / chipsize[0]

vo.process_frame(f0)
vo.process_frame(f1)

pairs = vo.temporal_match(f0, f1)
r = vo.pe.estimate(f1.kp, f0.kp, [ (b,a) for (a,b) in pairs ], False)

def img_err(f0, f1, RT):
  # Each point in f1's point cloud, compute xyz
  # Transform those points from f1's frame into f0's
  # Compute the uvd of each point
  # Sample f0 image using the uv

  def f_uvd(f):
    d_str = "".join([ chr(min(x / 4, 255)) for x in f.disp_values])
    d = vop.array(Image.fromstring("L", (640,480), d_str).resize(chipsize, Image.NEAREST).tostring())
    f_valid = d != 255.0
    f_d = d / 4
    u = vop.duplicate(vop.arange(chipsize[0]), chipsize[1]) * factor
    v = vop.replicate(vop.arange(chipsize[1]), chipsize[0]) * factor
    i = vop.array(Image.fromstring("L", (640,480), f.rawdata).resize(chipsize, Image.NEAREST).tostring()) / 255.
    return (f_valid, u, v, f_d, i)

  f0p,f0u,f0v,f0d,f0i = f_uvd(f0)
  f1p,f1u,f1v,f1d,f1i = f_uvd(f1)
  (f1x,f1y,f1z) = stereo_cam.pix2cam(f1u, f1v, f1d)
  (f1x0,f1y0,f1z0) = xform(RT, f1x, f1y, f1z)
  (f1u0,f1v0,f1d0) = stereo_cam.cam2pix(f1x0,f1y0,f1z0)
  addr = vop.floor(f1u0 / factor) + chipsize[0] * vop.floor(f1v0 / factor)
  ok = f1p & (0 <= f1u0) & (f1u0 < 640) & (0 <= f1v0) & (f1v0 < 480)
  addr = vop.where(ok, addr, 0)
  f1i0 = vop.take(f0i, addr)
  return (ok, f1p, f1i0, f1i)

def img_rms(f0, f1, RT):
  (ok, f1p, f1i0, f1i) = img_err(f0, f1, RT)
  diff = (f1i0 - f1i)
  error = diff * diff
  rms = math.sqrt(vop.where(ok, error, 0.0).sum() / vop.where(f1p, 1, 0).sum())
  return rms

def img_img(f0, f1, RT):
  (ok, f1p, f1i0, f1i) = img_err(f0, f1, RT)
  c_r = 255 * vop.where(ok, f1i, 0)
  c_g = 255 * vop.where(ok, f1i0, 0)
  c_b = 0 * c_g # 255 * vop.where(ok & (abs(f1d0 - f0d) < 1.5), 1.0, 0.0)
  return Image.merge("RGB", [Image.fromstring("L", chipsize, c.tostring()) for c in [c_r, c_g, c_b]])


if 1:
  started = time.time()
  niters = 10
  for i in range(niters):
    r = vo.pe.estimate(f1.kp, f0.kp, [ (b,a) for (a,b) in pairs ], False)
  print "Original took", 1e3 * (time.time() - started) / niters, "ms"
print "original", r
(inl, R,T) = r
R = list(R)
RT = R[0:3] + [T[0]] + R[3:6] + [T[1]] + R[6:9] + [T[2]]
img_img(f0, f1, RT).save("pe0.png")
print "original error", img_rms(f0, f1, RT)

(inliers, pose) = mype(stereo_cam, f0.kp, f1.kp, pairs)
print (inliers, pose)

niters = 100
started = time.time()
for i in range(niters):
  mype(stereo_cam, f0.kp, f1.kp, pairs)
print "Took", 1e3 * (time.time() - started) / niters, "ms"
sys.exit(0)

R = list(R)
RT = R[0:3] + [T[0]] + R[3:6] + [T[1]] + R[6:9] + [T[2]]
img_img(f0, f1, RT).save("pe.png")
Reuler = transformations.euler_from_rotation_matrix(numpy.array(R).reshape(3,3))

from math import *
x = numpy.arange(0,6e-2,6e-2/30)  
A,k,theta = 10, 1.0/3e-2, pi/6  
y_true = A*numpy.sin(2*pi*k*x+theta)
y_meas = y_true # + 2 * pylab.randn(len(x))

def residuals(p, y, x, f0, f1):  
  A,k,theta = p  
  err = y-A*numpy.sin(2*pi*k*x+theta)  
  e = (err*err).sum()
  return numpy.array([e]*6)

def peval(x, p):  
        return p[0]*numpy.sin(2*pi*p[1]*x+p[2])  

p0 = [ 8, 1.2/3e-2, pi/5 ] 

from scipy.optimize import leastsq  
if 1:
  plsq = leastsq(residuals, p0, args=(y_meas, x, f0, f1), full_output = 1)  
  print 
  print plsq
  print
  print "start         ", p0
  print "leastsq result", plsq[0]  
  pylab.plot(x, y_true, c='g')
  pylab.plot(x, peval(x, plsq[0]), c='b')
  pylab.show()

def p_to_rt(p):
  Reuler = p[:3]
  T = p[3:]
  R = list(transformations.rotation_matrix_from_euler(Reuler[0], Reuler[1], Reuler[2], 'sxyz')[:3,:3].reshape(9))
  RT = R[0:3] + [T[0]] + R[3:6] + [T[1]] + R[6:9] + [T[2]]
  return RT

def myfunc(p, f0, f1):
  RT = p_to_rt(p)
  e = img_rms(f0, f1, RT)
  return numpy.array([e]*6)

p0 = Reuler + T
print "p0", p0

print "before optimization error =", myfunc(p0, f0, f1)

plsq = leastsq(myfunc, p0, args=(f0, f1), full_output = 1, epsfcn = 1e-4)  
print
print plsq
print
print "start         ", p0
print "leastsq result", plsq[0]  
print "error improved from", myfunc(p0, f0, f1)[0], "to", myfunc(plsq[0], f0, f1)[0]

img_img(f0, f1, p_to_rt(plsq[0])).save("pe1.png")

