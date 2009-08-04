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

import sys
import time
import getopt
import math
import Queue
import threading
import random
import os

import rospy
import cv

import sensor_msgs.msg

def msg2rgb(msg):
    ma = msg.uint8_data # MultiArray
    dim = dict([ (d.label,d.size) for d in ma.layout.dim ])
    (w,h) = (dim['width'], dim['height'])
    im = cv.CreateImageHeader((w,h), cv.IPL_DEPTH_8U, dim['channel'])
    cv.SetData(im, ma.data, w)

    rgb = cv.CreateImage((w,h), cv.IPL_DEPTH_8U, 3)
    cv.CvtColor(im, rgb, cv.CV_BayerBG2RGB)
    return rgb

class snapshot:

  def __init__(self):
    rospy.Subscriber('/forearm/image_raw', sensor_msgs.msg.Image, self.handle_image_raw)
    self.counter = 0
    cv.NamedWindow("snap")

  def handle_image_raw(self, msg):

    print os.getcwd()
    if 1:
        self.counter += 1
        if self.counter > 0:
            rgb = msg2rgb(msg)
            cv.SaveImage("/tmp/snapshot.png", rgb)
            os.system("rosrun vision_tests killmaster.sh")
            rospy.core.signal_shutdown('completed')
    else:
        rgb = msg2rgb(msg)
        cv.ShowImage("snap", rgb)
        cv.WaitKey(10)


def main(args):
  time.sleep(3)
  s = snapshot()
  rospy.init_node('snapshot')
  rospy.spin()

if __name__ == '__main__':
  main(sys.argv)
