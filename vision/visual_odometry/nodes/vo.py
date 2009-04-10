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

import roslib
roslib.load_manifest('visual_odometry')

import sys
import time
import getopt

from math import *

import rospy

import image_msgs.msg

from stereo import DenseStereoFrame, SparseStereoFrame
import robot_msgs.msg

from visualodometer import VisualOdometer, FeatureDetectorHarris, FeatureDetector4x4, FeatureDetectorFast, Pose, DescriptorSchemeCalonder
import camera

class imgAdapted:
  def __init__(self, i, size):
    self.i = i
    self.size = size
  def tostring(self):
    return self.i.uint8_data.data

class dcamImage:
  def __init__(self, m):
    ma = m.byte_data # MultiArray

    self.data = ma.data
    d = ma.layout.dim
    assert d[0].label == "height"
    assert d[1].label == "width"
    self.size = (d[1].size, d[0].size)

  def tostring(self):
    return self.data

class VO:

  def __init__(self):
    rospy.Subscriber('/stereo/raw_stereo', image_msgs.msg.RawStereo, self.handle_raw_stereo, queue_size=2, buff_size=7000000)

    self.pub_vo = rospy.Publisher("/vo", robot_msgs.msg.VOPose)

    self.vo = None
    self.modulo = 0
    self.fd = FeatureDetectorFast(300)
    self.ds = DescriptorSchemeCalonder()

  def handle_raw_stereo(self, msg):
    size = (msg.left_info.width, msg.left_info.height)
    if self.vo == None:
      cam = camera.StereoCamera(msg.right_info)
      self.vo = VisualOdometer(cam, scavenge = False, 
                          inlier_error_threshold = 3.0, sba = None,
                          inlier_thresh = 100,
                          position_keypoint_thresh = 0.2, angle_keypoint_thresh = 0.15)
    pair = [imgAdapted(i, size) for i in [ msg.left_image, msg.right_image ]]
    af = SparseStereoFrame(pair[0], pair[1], feature_detector = self.fd, descriptor_scheme = self.ds)
    pose = self.vo.handle_frame(af)
    p = robot_msgs.msg.VOPose()
    p.inliers = self.vo.inl
    # XXX - remove after camera sets frame_id
    p.header = roslib.msg.Header(0, msg.header.stamp, "stereo_link")
    p.pose = robot_msgs.msg.Pose(robot_msgs.msg.Point(*pose.xform(0,0,0)), robot_msgs.msg.Quaternion(*pose.quaternion()))
    self.pub_vo.publish(p)

def main(args):
  vod = VO()

  rospy.init_node('vo')
  rospy.spin()
  vod.vo.summarize_timers()

if __name__ == '__main__':
  main(sys.argv)
