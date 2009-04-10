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
from vis import Vis
from picturemap import Picture, PictureMap
import pytf_swig

import image_msgs.msg
import robot_msgs.msg
import tf.msg
import deprecated_msgs.msg

import camera
import rospy
import Image

import time
import math

import vslam.msg

def py_transform_from_transform_stamped(transform):
    rot = transform.transform.rotation
    tr = transform.transform.translation
    ptf = pytf_swig.pyTransform()
    ptf.qx = rot.x
    ptf.qy = rot.y
    ptf.qz = rot.z
    ptf.qw = rot.w
    ptf.x  = tr.x
    ptf.y  = tr.y
    ptf.z  = tr.z
    ptf.frame_id = transform.header.frame_id
    ptf.parent_id = transform.parent_id
    ptf.stamp = transform.header.stamp.to_seconds()
    return ptf

class PicmapNode:
  def __init__(self, args):
    rospy.init_node('picmap_server')
    stereo_cam = camera.Camera((389.0, 389.0, 89.23 * 1e-3, 323.42, 323.42, 274.95))

    self.vo = None
    self.transformer = pytf_swig.pyTransformer()
    self.transformer.setExtrapolationLimit(0.0)

    #self.pub = rospy.Publisher("/picmap_pose", vslam.msg.Picmap)

    rospy.Subscriber('/stereo/raw_stereo', image_msgs.msg.RawStereo, self.handle_raw_stereo, queue_size=2, buff_size=7000000)
    rospy.Subscriber('/amcl_pose', robot_msgs.msg.PoseWithCovariance, self.handle_amcl_pose)
    rospy.Subscriber('/tf_message', tf.msg.tfMessage, self.handle_tf)

  def handle_tf(self, msg):
    print "incoming transform", min([ t.header.stamp.to_seconds() for t in msg.transforms ])
    for transform in msg.transforms:
      ptf = py_transform_from_transform_stamped(transform)
      self.transformer.setTransform(ptf)

  def handle_raw_stereo(self, msg):
    print "incoming picture  ", msg.header.stamp.to_seconds()
    size = (msg.left_info.width, msg.left_info.height)
    cam = camera.StereoCamera(msg.right_info)
    if self.vo == None:
      self.fd = FeatureDetectorFast(300)
      self.ds = DescriptorSchemeCalonder()
      self.vo = VisualOdometer(cam, scavenge = False,
                          inlier_error_threshold = 3.0, sba = None,
                          inlier_thresh = 100,
                          position_keypoint_thresh = 0.2, angle_keypoint_thresh = 0.15)
      self.keys = set()
      self.v = Vis('raw stereo')
      self.pm = PictureMap(self.ds)

    pair = [Image.fromstring("L", size, i.uint8_data.data) for i in [ msg.left_image, msg.right_image ]]
    af = SparseStereoFrame(pair[0], pair[1], feature_detector = self.fd, descriptor_scheme = self.ds)
    af.t = msg.header.stamp.to_seconds()
    self.v.show(msg.left_image.uint8_data.data, [])

    self.vo.handle_frame(af)

    k = self.vo.keyframe
    if (not (k.id in self.keys)):
      self.keys.add(k.id)
      picture = Picture(k.features(), k.descriptors())
      self.pm.newpic(k.t, cam, picture, True)

    picture = Picture(af.features(), af.descriptors())
    self.pm.newpic(msg.header.stamp.to_seconds(), cam, picture, False)

    self.pm.resolve(self.transformer)

  def handle_amcl_pose(self, msg):
    if self.vo:
      happy = max([msg.covariance[0], msg.covariance[7]]) < 0.003
      print "picmap node got amcl", msg.covariance[0], msg.covariance[7], "happy", happy
      self.pm.newLocalization(msg.header.stamp.to_seconds(), happy)

def main(args):
  rms = PicmapNode(args)
  rospy.spin()

if __name__ == '__main__':
  main(sys.argv)
