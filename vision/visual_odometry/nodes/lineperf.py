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

from vis import Vis

from math import *

from std_msgs.msg import Image, ImageArray, String, VisualizationMarker
from cv_view.msg import Line, Lines
from visual_odometry.msg import Frame, Pose44, Keypoint, Descriptor
import rospy
from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, FeatureDetectorHarris
import camera
from marker import Marker
import timer

import PIL.Image
import PIL.ImageDraw
#import pickle
import cPickle as pickle
import random

import pylab

class as_lines:
  type = Lines
  def encode(self, x):
    return Lines([ Line(l, r, g, b, x0, y0, x1, y1) for (l, r, g, b, x0, y0, x1, y1) in x ])
  def decode(self, x):
    return x

class as_string:
  type = String
  def encode(self, x):
    return String(pickle.dumps(x))
  def decode(self, x):
    return Lines([ Line(l, r, g, b, x0, y0, x1, y1) for (l, r, g, b, x0, y0, x1, y1) in pickle.loads(x.data) ])

mydata = as_lines()

class lineperftest:

  def __init__(self, mode):
    if mode == 'send':
      rospy.Subscriber('/videre/images', ImageArray, self.display_array)
      rospy.Subscriber('/videre/cal_params', String, self.display_params)
      self.viz_pub = rospy.Publisher("/overlay", mydata.type)
    else:
      rospy.Subscriber("/overlay", mydata.type, self.handle_lines, buff_size = 65536)

    self.times = []

  def display_params(self, iar):
    cam = camera.VidereCamera(iar.data)

  def display_array(self, iar):
    points = [ (random.randrange(640), random.randrange(480)) for i in range(400) ]
    ls = []
    lr = "left_rectified"
    ls = [ (lr, 0,255,0,  x,y-2,x,y+2) for (x,y) in points]
    ls += [ (lr, 0,255,0, x-2,y,x+2,y) for (x,y) in points]
    started = time.time()
    w = mydata.encode(ls)
    self.viz_pub.publish(w)
    took = time.time() - started
    print took
    if took < 0.030:
      self.times.append(took)

  def handle_lines(self, iar):
    d = mydata.decode(iar)
    print "got %d lines" % len(d.lines)

  def report(self):
    pylab.hist(self.times, 100, (0.0, 0.030))
    pylab.show()

def main(args):

  vod = lineperftest(args[1])

  rospy.ready('lineperf_%s' % args[1])
  rospy.spin()
  if args[1] == 'send':
    vod.report()

if __name__ == '__main__':
  main(sys.argv)
