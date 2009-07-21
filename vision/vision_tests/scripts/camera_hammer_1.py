#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import roslib
roslib.load_manifest('vision_tests')
import rospy

import sys
import math
import time
from math import *

import cv
import dcam

def compose(a, b, d):
    w,h = (640,480)
    r = cv.CreateImage((w*3, h), cv.IPL_DEPTH_8U, 3)
    cv.SetZero(r)
    
    for (i, src) in enumerate([a, b, d]):
        if src != None:
          cv.SetImageROI(r, (w*i, 0, w, h))
          if src.nChannels == 1:
            deep = cv.CreateImage((w, h), cv.IPL_DEPTH_8U, 3)
            cv.CvtColor(src, deep, cv.CV_GRAY2BGR)
            cv.Copy(deep, r)
          else:
            cv.Copy(src, r)
    cv.ResetImageROI(r)
    return r

def saveImage(dcim, filename):
    w,h,l,r,d = dcim

    cv_l = None
    cv_r = None
    cv_d = None

    if l != None:
      channels = len(l) / (w * h)
      cv_l = cv.CreateImageHeader((w, h), cv.IPL_DEPTH_8U, channels)
      cv.SetData(cv_l, str(l), w * channels)
      if channels == 3:
        cv.CvtColor(cv_l, cv_l, cv.CV_BGR2RGB)

    if r != None:
      channels = len(r) / (w * h)
      cv_r = cv.CreateImageHeader((w, h), cv.IPL_DEPTH_8U, channels)
      cv.SetData(cv_r, str(r), w * channels)
      if channels == 3:
        cv.CvtColor(cv_r, cv_r, cv.CV_BGR2RGB)

    if d != None:
      (dimg, dpp, numDisp) = d
      max_disparity = (dpp * numDisp) - 1
      d16 = cv.CreateImageHeader((w, h), cv.IPL_DEPTH_16U, 1)
      cv.SetData(d16, str(dimg), w * 2)
      cv_d = cv.CreateImage((w, h), cv.IPL_DEPTH_8U, 1)
      cv.CvtScale(d16, cv_d, 255.0 / max_disparity)

    cv.SaveImage(filename, compose(cv_l, cv_r, cv_d))

def runTest(seq, mode, fps):
    print mode
    dc = dcam.dcam(mode, fps)
    paramstr = dc.retParameters()

    stamp = []
    for i in range(35):
        print i
        dcim = dc.getImage()
        stamp.append(time.time())
        #saveImage(dcim, "cam-%s-%04d-%03d.png" % (mode, seq, i))
    saveImage(dcim, "cam-%s-%04d.png" % (mode, seq))

    # Might be some warmup, so only consider the last 15 frames for frame rate calc
    stamp = stamp[-15:]
    actual_fps = (len(stamp) - 1) / (stamp[-1] - stamp[0])
    if abs(fps - actual_fps) > 1.0:
        print 'ERROR', 'fps', fps, 'actual_fps', actual_fps

seq = int(sys.argv[1])
m = sys.argv[2]
fps = int(sys.argv[3])

runTest(seq, m, fps)

## Collect goldens
#modes = ('none', 'test')
#goldens = {}
#for m in modes:
#    goldens[m] = runTest(m)
#
#modes = ('test',)
#for i in range(10):
#    for m in modes:
#        gl,gr = goldens[m]
#        l,r = runTest(m)
#        comp = compose(l, r)
#        cv.SaveImage("cam-%s-%06d.png" % (m, i), compose(l, r))
