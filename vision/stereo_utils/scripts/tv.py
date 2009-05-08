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
roslib.load_manifest('stereo_utils')
import rospy

import sys
from stereo_utils.reader import CVreader
import cv
import time

def playlist(args):
  for f in args:
    r = CVreader(f)
    for d in r:
      yield d + (f,)

framecounter = 0
font = cv.InitFont(cv.CV_FONT_HERSHEY_SIMPLEX, 1, 1)
cv.NamedWindow("tv", 1)

for cam,l_image,r_image,label in playlist(sys.argv[1:]):
  ipl = l_image
  msg = "%4d" % framecounter
  (w,h),baseline = cv.GetTextSize(msg, font)
  (_, imageh) = cv.GetSize(l_image)

  # Gray out the top-left rectangle
  cv.SetImageROI(ipl, (0, imageh+baseline, w + 1, imageh))
  cv.ConvertScale(ipl, ipl, 0.75)
  cv.ResetImageROI(ipl)

  # Draw the time
  cv.PutText(ipl, msg, (0,imageh+baseline), font, (255,255,255))

  cv.ShowImage("tv", ipl)

  cmd = cv.WaitKey(10)
  # Space is pause
  if cmd == ord(" "):
    cv.WaitKey()
  if cmd in [ 27, ord('q') ]:
    break

  framecounter += 1
