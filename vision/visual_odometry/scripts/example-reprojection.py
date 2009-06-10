import roslib
roslib.load_manifest('visual_odometry')
import rospy
import Image
import math

import os

from stereo_utils import camera
from stereo_utils.stereo import SparseStereoFrame, DenseStereoFrame
from stereo_utils.descriptor_schemes import DescriptorSchemeCalonder, DescriptorSchemeSAD
from stereo_utils.feature_detectors import FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
from visual_odometry.visualodometer import VisualOdometer, Pose
from visual_odometry.pe import PoseEstimator
import vop
import cv

fd = FeatureDetectorStar(300)
ds = DescriptorSchemeSAD()

from stereo_utils.reader import reader
#r = reader("/u/jamesb/sequences/kk_2009-02-24-17-24-55-topic") 
r = reader("/u/jamesb/sequences/kk_2009-02-24-17-22-41-topic") 

#(i0,i1) = [ (Image.open("f%d-left.png" % i), Image.open("f%d-right.png" % i)) for i in [0, 1] ]

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

def vop2iplimage(v):
  ipl = cv.CreateImage((640,480), 8, 1)
  cv.SetData(ipl, (v * 255).tostring(), 640)
  return ipl

def img_img(f0, f1, RT, stereo_cam):
  (ok, f1p, f1i0, f1i) = img_err(f0, f1, RT, stereo_cam)
  c_r = vop.where(ok, f1i, 0)
  c_g = vop.where(ok, f1i0, 0)
  return c_r, c_g

cv.NamedWindow("disparity", 1)

cv.NamedWindow("merged", 1)
cv.MoveWindow("merged", 0, 0)
cv.NamedWindow("diff", 1)
cv.MoveWindow("diff", 700, 0)
chain = []
for cam,L,R in r:
  f1 = DenseStereoFrame(L, R, feature_detector = fd, descriptor_scheme = ds)
  if chain == []:
    vo = VisualOdometer(cam, ransac_iters = 500)
  vo.handle_frame(f1)
  d_str = "".join([ chr(min(x / 4, 255)) for x in f1.disp_values])
  disparity = cv.CreateImage((640,480), 8, 1)
  cv.SetData(disparity, d_str, 640)
  cv.ShowImage("disparity", disparity)
  if len(chain) > 5:
    f0 = chain[0]
    img = cv.CreateImage((640,480), 8, 3)
    diffpose = ~f0.pose * f1.pose
    a,b = img_img(f0, f1, diffpose.tolist(), cam)
    ai = vop2iplimage(a)
    bi = vop2iplimage(b)
    # a is red
    # b is green/blue
    cv.Merge(bi, bi, ai, None, img)
    cv.ShowImage("merged", img)
    di = vop2iplimage(abs(a - b))
    cv.Threshold(di, di, 20.0, 255.0, cv.CV_THRESH_BINARY)
    cv.Erode(di, di)
    if 0:
      cv.Dilate(di, di)
    if 0:
      rgbdi = cv.CreateImage((640,480), 8, 3)
      cv.Merge(di, di, di, None, rgbdi)
      if 0:
        li = cv.HoughLines2(di,
                            cv.CreateMemStorage(),
                            cv.CV_HOUGH_PROBABILISTIC,
                            1,
                            math.pi/180,
                            50,
                            200,
                            200)
        for a,b in li:
          cv.Line(rgbdi, a, b, (255,0,0), 3, 8)
      else:
        li = cv.HoughLines2(di,
                            cv.CreateMemStorage(),
                            cv.CV_HOUGH_STANDARD,
                            1,
                            math.pi/180,
                            100,
                            0,
                            0)
        for rho,theta in li:
          a = math.cos(theta)
          b = math.sin(theta)
          x0 = a * rho
          y0 = b * rho
          cv.Line(rgbdi, (x0 + 1000*-b, y0+1000*a), (x0 - 1000*-b, y0-1000*a), (255,0,0), 3, 8)
      cv.Merge(None, None, di, None, rgbdi)
      cv.ShowImage("diff", rgbdi)
    elif 1:
      rgbdi = cv.CreateImage((640,480), 8, 3)
      cv.Merge(di, di, di, None, rgbdi)
      contours = cv.FindContours(di, cv.CreateMemStorage(), cv.CV_RETR_CCOMP, cv.CV_CHAIN_APPROX_SIMPLE)
      def cwalker(contour):
        while contour:
          yield contour
          contour = contour.h_next()
      s_contours = [ (abs(cv.ContourArea(c)),c) for c in cwalker(contours) ]
      def aspect(c):
        ((x,y),(w,h),th) = cv.MinAreaRect2(c, cv.CreateMemStorage())
        return max([w/h, h/w])
      if 1:
        big_contours = [ c for a,c in s_contours if (a > 100) and (aspect(c) < 5.0) ]
        for c in big_contours:
          cv.FillPoly(rgbdi, [ c ], (255,0,0))
        all_rects = [ cv.BoundingRect(c) for c in big_contours ]
        if all_rects != []:
          mr = all_rects[0]
          for r in all_rects[1:]:
            mr = cv.MaxRect(mr, r)
          (x,y,w,h) = mr
          cv.Rectangle(rgbdi, (x,y), (x+w,y+h), cv.RGB(255,0,0))

      cv.ShowImage("diff", rgbdi)
    else:
      cv.ShowImage("diff", di)
    cv.WaitKey(10)
  chain.append(f1)
  chain = chain[-6:]

"""
  if 1:
    img = cv.CreateImage((640,480), 8, 3)
    a,b = img_img(f0, f1, vo.pose.tolist())
    d = vop.abs(a-b)
    a = vop2iplimage(a)
    b = vop2iplimage(b)
    # a is red
    # b is green/blue
    cv.Merge(b, b, a, None, img)
    cv.ShowImage("snap", img)
    cv.WaitKey()
"""
