import rostools
rostools.update_path('visual_odometry')
import rostest
import rospy

import vop

import math
import sys
import time

sys.path.append('lib')
import votools

import Image

import transformations
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
#(f0,f1) = [ ComputedDenseStereoFrame(Image.open("f%d-left.png" % i), Image.open("f%d-right.png" % i)) for i in [0, 1] ]
(f0,f1) = [ ComputedDenseStereoFrame(Image.open("../vslam/trial/%06dL.png" % i), Image.open("../vslam/trial/%06dR.png" % i)) for i in [276, 278] ]
#(f0,f1) = [ ComputedDenseStereoFrame(Image.open("../vslam/trial/%06dL.png" % i), Image.open("../vslam/trial/%06dR.png" % i)) for i in [277, 278] ]
#(f0,f1) = [ ComputedDenseStereoFrame(Image.open("../vslam/trial/%06dL.png" % i), Image.open("../vslam/trial/%06dR.png" % i)) for i in [0, 1] ]

chipsize = (640,480)
factor = 640 / chipsize[0]

vo.process_frame(f0)
vo.process_frame(f1)

pairs = vo.temporal_match(f0, f1)
r = vo.pe.estimate(f1.kp, f0.kp, [ (b,a) for (a,b) in pairs ], False)

ransac_size = 100
picks = [ random.sample(range(len(pairs)), 3) for i in range(ransac_size) ]

def vop3(L):
  x0 = vop.array([ x for (x,_,_) in L ])
  y0 = vop.array([ y for (_,y,_) in L ])
  z0 = vop.array([ z for (_,_,z) in L ])
  return (x0, y0, z0)
  
def xform(M, x, y, z):
  nx = vop.mad(M[0], x, vop.mad(M[1], y, vop.mad(M[2], z, M[3])))
  ny = vop.mad(M[4], x, vop.mad(M[5], y, vop.mad(M[6], z, M[7])))
  nz = vop.mad(M[8], x, vop.mad(M[9], y, vop.mad(M[10], z, M[11])))
  return (nx, ny, nz)

def mype(cam, cp1, cp0, pairs):
  p0 = [ stereo_cam.pix2cam(*p) for p in cp0 ]
  xyz0 = [ p0[i] for (j,i) in pairs ]

  x0 = vop.array([ x for (x,y,z) in xyz0 ])
  y0 = vop.array([ y for (x,y,z) in xyz0 ])
  z0 = vop.array([ z for (x,y,z) in xyz0 ])

  uvd1 = [ cp1[j] for (j,i) in pairs ]
  u1 = vop.array([ u for (u,v,d) in uvd1 ])
  v1 = vop.array([ v for (u,v,d) in uvd1 ])
  d1 = vop.array([ d for (u,v,d) in uvd1 ])

  (p0_x, p0_y, p0_z) = stereo_cam.pix2cam(*vop3(cp0))
  (p1_x, p1_y, p1_z) = stereo_cam.pix2cam(*vop3(cp1))

  best = (0, None)

  for ransac in range(ransac_size):
    #triple = random.sample(pairs, 3)
    triple = [pairs[i] for i in picks[ransac]]

    # Takes 3ms
    p0x = [ p0_x[i] for (_,i) in triple ]
    p0y = [ p0_y[i] for (_,i) in triple ]
    p0z = [ p0_z[i] for (_,i) in triple ]
    p0s = p0x+p0y+p0z
    p1x = [ p1_x[i] for (i,_) in triple ]
    p1y = [ p1_y[i] for (i,_) in triple ]
    p1z = [ p1_z[i] for (i,_) in triple ]
    p1s = p1x+p1y+p1z

    # Takes 3ms
    R,T,RT = votools.SVD(p0s, p1s)


    # Check Cont inliers for RT against xyz0 -> uvd0 vs uvd1
    # Takes 15
    (u0,v0,d0) = stereo_cam.cam2pix(*xform(RT, x0, y0, z0))
    pred_inl = vop.where(vop.maximum(vop.maximum(abs(u0 - u1), abs(v0 - v1)), abs(d0 - d1)) > 3.0, 0.0, 1.0)
    inliers = int(pred_inl.sum())
    if inliers > best[0]:
      best = (inliers, (R,T))
  return best


  if 0:
    import pylab
    pylab.scatter(g_inl, g_rms)
    for i in range(len(g_inl)):
      pylab.annotate('%d' % i, (g_inl[i], g_rms[i]))
    pylab.show()


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
    r = vo.pe.estimate(f1.kp, f0.kp, [ (b,a) for (a,b) in pairs ], True)
  print "Original took", 1e3 * (time.time() - started) / niters, "ms"
print "original", r
(inl, R,T) = r
R = list(R)
RT = R[0:3] + [T[0]] + R[3:6] + [T[1]] + R[6:9] + [T[2]]
img_img(f0, f1, RT).save("pe0.png")
print "original error", img_rms(f0, f1, RT)

(inliers, (R,T)) = mype(stereo_cam, f0.kp, f1.kp, pairs)
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

sys.exit(0)
niters = 10
started = time.time()
for i in range(niters):
  mype(stereo_cam, f0.kp, f1.kp, pairs)
print "Took", 1e3 * (time.time() - started) / niters, "ms"
