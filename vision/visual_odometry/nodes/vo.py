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

import rospy
from std_msgs.msg import Image, ImageArray, String, VisualizationMarker
from robot_msgs.msg import VOPose
import std_msgs.msg as stdmsg
from image_msgs.msg import StereoInfo, Image, CamInfo
from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer, FeatureDetectorHarris, FeatureDetector4x4, FeatureDetectorFast, Pose
import camera
from threading import Lock

class imgAdapted:
  def __init__(self, i):
    self.i = i
    self.size = (i.width, i.height)
  def tostring(self):
    return self.i.data

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
    rospy.TopicSub('/videre/images', ImageArray, self.handle_array, queue_size=2, buff_size=7000000)
    rospy.TopicSub('/videre/cal_params', String, self.handle_params)
    #rospy.TopicSub('/dcam/stereo_info', StereoInfo, self.handle_stereo_info)
    rospy.TopicSub('/dcam/right/cam_info', CamInfo, self.handle_right_cam_info)
    rospy.TopicSub('/dcam/left/image_rect', Image, self.handle_left_image, queue_size=2, buff_size=7000000)
    rospy.TopicSub('/dcam/right/image_rect', Image, self.handle_right_image, queue_size=2, buff_size=7000000)

    self.collected = {}
    self.collection_lock = Lock()

    self.pub_vo = rospy.Publisher("/vo", VOPose)

    self.vo = None
    self.modulo = 0

  # Old stereo node (dcam)

  def handle_params(self, iar):
    if not self.vo:
      cam = camera.VidereCamera(iar.data)
      self.vo = VisualOdometer(cam)

  def handle_array(self, iar):
    if self.vo:
      imgR = imgAdapted(iar.images[0])
      imgL = imgAdapted(iar.images[1])
      af = SparseStereoFrame(imgL, imgR)
      self.handle_frame(iar.header.stamp, af)

  def handle_frame(self, stamp, af):
      if 1:
        pose = self.vo.handle_frame(af)
      else:
        pose = Pose()
      #print self.vo.num_frames, pose.xform(0,0,0), pose.quaternion()
      p = VOPose()
      p.inliers = self.vo.inl
      # XXX - remove after camera sets frame_id
      p.header = rostools.msg.Header(0, stamp, "stereo_link")
      p.pose = stdmsg.Pose(stdmsg.Point(*pose.xform(0,0,0)), stdmsg.Quaternion(*pose.quaternion()))
      self.pub_vo.publish(p)

  def push(self, tag, m):
    t = m.header.stamp.secs + m.header.stamp.nsecs * 1e-9
    self.collected.setdefault(t, {})
    assert not tag in  self.collected[t]
    self.collected[t][tag] = m

  def handle_pair(self, l, r):
    if self.vo:
      af = SparseStereoFrame(dcamImage(l), dcamImage(r))
      self.handle_frame(l.header.stamp, af)

  def check_queues(self, handler):
    want = ['L', 'R']
    completed = set()
    for t in sorted(self.collected.keys()):
      if set(self.collected[t].keys()) == set(want):
        completed.add(t)

    for t in sorted(completed):
      v = self.collected[t]
      handler(*tuple([v[l] for l in want]))
      print "deliver", t, self.collected[t].keys()

    if len(completed) != 0:
      for t in self.collected.keys():
        if t <= max(completed):
          del self.collected[t]

  # New stereo node (dcam)

  def handle_stereo_info(self, m):
    print "stereo info", m.T, m.RP

  def handle_right_cam_info(self, m):
    if not self.vo:
      cam = camera.StereoCamera(m)
      self.vo = VisualOdometer(cam)
      self.intervals = []
      self.took = []

  def handle_left_image(self, m):
    self.collection_lock.acquire()
    self.push('L', m)
    self.check_queues(self.handle_pair)
    self.collection_lock.release()

  def handle_right_image(self, m):
    self.collection_lock.acquire()
    self.push('R', m)
    self.check_queues(self.handle_pair)
    self.collection_lock.release()

  def dump(self):
    iv = self.intervals
    took = self.took
    print "Incoming frames: %d in %f, so %fms" % (len(iv), iv[-1] - iv[0], (iv[-1] - iv[0]) / len(iv))
    print "Time in callback: %fms" % (sum(took) / len(took))

def main(args):
  vod = VO()

  rospy.ready('vo')
  rospy.spin()
  vod.vo.summarize_timers()
  vod.dump()

if __name__ == '__main__':
  main(sys.argv)
