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
roslib.load_manifest('vslam')

import sys
import time
import getopt

from math import *

import rospy
import image_msgs.msg
import std_msgs.msg

from reader import reader

def main(args):
  rospy.init_node('play')

  pub = rospy.Publisher("/stereo/raw_stereo", image_msgs.msg.RawStereo)
  r = reader(args[1])
  f = 20543
  r.seek(f)
  for cam,l_image,r_image in r.next():
    (Fx, Fy, Tx, Clx, Crx, Cy) = cam.params
    p = image_msgs.msg.RawStereo()

    w,h = l_image.size
    p.stereo_info.width = w
    p.stereo_info.height = h
    p.stereo_info.T = [ 0.0 ] * 3
    p.stereo_info.Om = [ 0.0 ] * 3
    p.stereo_info.RP = [ 0.0 ] * 16

    for inf in [ p.left_info, p.right_info ]:
      inf.width = w
      inf.height = h
      inf.D = [ 0.0 ] * 5
      inf.K = [ 0.0 ] * 9
      inf.R = [ 0.0 ] * 9
      inf.P = [ 0.0 ] * 12

    p.has_disparity = 0

    for lr in [ p.left_image, p.right_image, p.disparity_image ]:
      lr.label = "none"
      lr.encoding = "1"
      lr.depth = "uint8"
      for d in [ lr.uint8_data, lr.int8_data, lr.uint16_data, lr.int16_data, lr.uint32_data, lr.int32_data, lr.uint64_data, lr.int64_data, lr.float32_data, lr.float64_data ]:
        d.layout.dim = []
        d.layout.data_offset = 0
        d.data = []
      lr.uint8_data.data = ""
    p.left_type = 4
    p.right_type = 4

    p.right_info.P = [ Fx, 0,  Clx, -Fx * Tx,
                       0,  Fy, Cy, 0,
                       0,  0,  1,  0 ]

    p.left_image.uint8_data.data = l_image.tostring()
    p.left_image.uint8_data.layout.dim =  [ std_msgs.msg.MultiArrayDimension("width", w, w), std_msgs.msg.MultiArrayDimension("height", h, h) ]
    p.left_image.uint8_data.layout.data_offset = 0
    p.right_image.uint8_data.data = r_image.tostring()
    p.right_image.uint8_data.layout.dim =  [ std_msgs.msg.MultiArrayDimension("width", w, w), std_msgs.msg.MultiArrayDimension("height", h, h) ]
    p.right_image.uint8_data.layout.data_offset = 0

    for d in [p.disparity_info]:
      d.height = 0
      d.width = 0
      d.dpp = 0
      d.num_disp = 0
      d.im_Dtop = 0
      d.im_Dleft = 0
      d.im_Dwidth = 0
      d.im_Dheight = 0
      d.corr_size = 0
      d.filter_size = 0
      d.hor_offset = 0
      d.texture_thresh = 0
      d.unique_thresh = 0
      d.smooth_thresh = 0
      d.speckle_diff = 0
      d.speckle_region_size = 0
      d.unique_check = 0

    pub.publish(p)
    time.sleep(0.2)
    f += 1
    if f == 26968:
      break

if __name__ == '__main__':
  main(sys.argv)
