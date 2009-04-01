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

from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, Pose, DescriptorSchemeCalonder, DescriptorSchemeSAD, FeatureDetectorFast, FeatureDetector4x4, FeatureDetectorStar, FeatureDetectorHarris
from skeleton import Skeleton
import image_msgs.msg
import deprecated_msgs.msg

import camera
import rospy
import Image

import time
import math

import vslam.msg

class RoadmapServer:
  def __init__(self, args):
    rospy.init_node('roadmap_server')
    stereo_cam = camera.Camera((389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95))
    self.skel = Skeleton(stereo_cam)
    if len(args) > 1:
      self.skel.load(args[1])
      self.skel.optimize()
      self.startframe = 100000
    else:
      self.startframe = 0

    self.vo = None

    self.pub = rospy.Publisher("/roadmap", vslam.msg.Roadmap)

    time.sleep(1)
    self.send_map()

    rospy.TopicSub('/stereo/raw_stereo', image_msgs.msg.RawStereo, self.handle_raw_stereo, queue_size=2, buff_size=7000000)

  def send_map(self):
    p = vslam.msg.Roadmap()
    (ns,es,lo) = self.skel.localization()
    print "ns,es,lo", ns, es, lo
    p.nodes = [ vslam.msg.Node(x,y,t) for (x,y,t) in ns ]
    p.edges = [ vslam.msg.Edge(a,b) for (a,b) in es ]
    p.localization = lo
    self.pub.publish(p)

  def handle_raw_stereo(self, msg):
    size = (msg.left_info.width, msg.left_info.height)
    if self.vo == None:
      cam = camera.StereoCamera(msg.right_info)
      self.vo = VisualOdometer(cam, scavenge = False, feature_detector = FeatureDetectorFast(),
                          descriptor_scheme = DescriptorSchemeCalonder(),
                          inlier_error_threshold = 3.0, sba = None,
                          inlier_thresh = 100,
                          position_keypoint_thresh = 0.2, angle_keypoint_thresh = 0.15)
      self.vo.num_frames = self.startframe
    pair = [Image.fromstring("L", size, i.uint8_data.data) for i in [ msg.left_image, msg.right_image ]]
    af = SparseStereoFrame(pair[0], pair[1])
    self.vo.handle_frame(af)
    if self.skel.add(self.vo.keyframe):
      self.send_map()

def dist(a,b):
  xd = a[0] - b[0]
  yd = a[1] - b[1]
  return math.sqrt(xd*xd + yd*yd)

class FakeRoadmapServer:

  def __init__(self, args):
    rospy.init_node('roadmap_server')
    self.pub = rospy.Publisher("/roadmap", vslam.msg.Roadmap)
    rospy.TopicSub('/localizedpose', deprecated_msgs.msg.RobotBase2DOdom, self.handle_localizedpose)
    self.nodes = []

  def handle_localizedpose(self, msg):
    print msg.pos.x, msg.pos.y, msg.pos.th
    n = (msg.pos.x, msg.pos.y, msg.pos.th)
    if self.nodes == [] or (dist(self.nodes[-1], n) > 1.0) or (abs(self.nodes[-1][2] - n[2]) > (2.0 * pi / 180)):
      self.nodes.append((msg.pos.x, msg.pos.y, msg.pos.th))
      self.send_map(msg.header.stamp)

  def send_map(self, stamp):
    p = vslam.msg.Roadmap()
    print "sending time", stamp
    p.header.stamp = stamp
    p.nodes = [ vslam.msg.Node(x,y,t) for (x,y,t) in self.nodes ]
    es = []
    for a in range(len(self.nodes)):
      for b in range(a):
        if dist(self.nodes[a], self.nodes[b]) < 1.5:
          es.append((a,b))
    print len(es), "edges"
    p.edges = [ vslam.msg.Edge(a,b) for (a,b) in es ]
    p.localization = len(self.nodes) - 1
    print "Stamp is ", stamp, " header stamp is ", p.header.stamp
    self.pub.publish(p)

def main(args):
  if args[1] == 'stage':
    rms = FakeRoadmapServer(args)
  else:
    rms = RoadmapServer(args)
  rospy.spin()

if __name__ == '__main__':
  main(sys.argv)
