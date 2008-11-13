#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
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

import rostools
rostools.update_path('visual_odometry')

import sys
import time
import getopt

from math import *

from std_msgs.msg import Image, ImageArray, String, VisualizationMarker
from cv_view.msg import Line, Lines
from visual_odometry.msg import Frame, Pose44, Keypoint, Descriptor
import rospy
from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, FeatureDetectorHarris
import camera
from marker import Marker

import PIL.Image
import PIL.ImageDraw
import pickle

SEE = 0

class imgAdapted:
  def __init__(self, i):
    self.i = i
    self.size = (i.width, i.height)
  def tostring(self):
    return self.i.data

class VO:
  def __init__(self):
    rospy.TopicSub('/videre/images', ImageArray, self.handle_array)
    rospy.TopicSub('/videre/cal_params', String, self.handle_params)

    self.vo_pub = rospy.Publisher("/vo/key", Frame)
    self.vo = None

  def handle_params(self, iar):
    if not self.vo:
      cam = camera.VidereCamera(iar.data)
      self.vo = VisualOdometer(cam)
      self.started = None
      self.previous_keyframe = None
      self.know_state = 'lost'

  def handle_array(self, iar):
    af = None
    if self.vo:
      imgR = imgAdapted(iar.images[0])
      imgL = imgAdapted(iar.images[1])
      af = SparseStereoFrame(imgL, imgR)

      pose = self.vo.handle_frame(af)
      print pose.xform(0,0,0)

def main(args):

  vod = VO()

  rospy.ready('vo')
  rospy.spin()

if __name__ == '__main__':
  main(sys.argv)
