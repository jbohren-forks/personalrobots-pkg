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
import rostest
import rospy

import vop

import sys

import Image as Image
import ImageChops as ImageChops
import ImageDraw as ImageDraw
import ImageFilter as ImageFilter

import random
import unittest
import math
import pickle
import time
import place_recognition

import numpy

from visualodometer import DescriptorSchemeCalonder, Pose
from pe import PoseEstimator
from skeleton import Skeleton
from reader import reader

from tf import transformations

def getpose(transformer, parent_frame, frame, ttime):
    ptf = transformer.getTransform(parent_frame, frame, ttime)
    p = Pose(transformations.rotation_matrix_from_quaternion((ptf.qx, ptf.qy, ptf.qz, ptf.qw))[:3,:3], numpy.array((ptf.x, ptf.y, ptf.z)))
    return p

class Timeline:
  def __init__(self, length):
    self.q = []
    self.length = length

  def add(self, time, datum):
    self.q.append((time, datum))
    while len(self.q) > self.length:
      oldest = min(self.q)
      self.q.remove(oldest)

  def query(self, time):
    earlier = [ (t,d) for (t,d) in self.q if t < time ]
    later = [ (t,d) for (t,d) in self.q if t > time ]
    if earlier != [] and later != []:
      return (max(earlier)[1], min(later)[1])
    else:
      return None

  def __len__(self):
    return len(self.q)

  def __iter__(self):
    return self.q.__iter__()

  def __delitem__(self, i):
    self.q.remove(i)

  def __repr__(self):
    return " ".join([ str(e[1]) for e in sorted(self.q) ])

class Picture:
  def __init__(self, keypoints, descriptors, pose = None):
    self.keypoints = keypoints
    self.descriptors = descriptors
    self.pose = pose

class PictureMap:
  """ Picture map """

  def __init__(self, ds):
    self.cameras = []
    self.pics = []
    filename = '/u/mihelich/images/holidays/holidays.tree'
    self.vt = place_recognition.load(filename)

    self.pe = PoseEstimator()

    self.clear()

    self.ds = ds

    self.verbose = 1

  def clear(self):
    self.localizationOK = Timeline(1000)
    self.observations = Timeline(30)

  def newLocalization(self, t, l):
    self.localizationOK.add(t, l)

  def isVerbosity(self, n):
    return n <= self.verbose

  def newpic(self, when, cam, picture, iskey):
    if self.isVerbosity(1):
      print "Adding picture", when
    self.observations.add(when, (cam, picture, iskey))

  def find(self, descriptors):
    """ Search the map for a camera's frame """
    r = self.vt.topN(None, descriptors, len(self.pics), False)
    best = min([ (d,i) for (i,d) in enumerate(r) ])[1]
    return best

  def resolve(self, transformer):

    if self.isVerbosity(1):
      print "resolver"
      print "getLatestCommonTime", transformer.getLatestCommonTime('map', 'stereo_link')
      if len(self.observations) > 0:
        print "obs earliest       ", min(self.observations)[0]
        print "obs latest         ", max(self.observations)[0]

    for (t,(cam,picture,iskey)) in self.observations:
      # Add keys to the map if they are well-localized and TF can transform them
      print iskey, transformer.canTransform('map', 'stereo_link', t)
      if iskey and transformer.canTransform('map', 'stereo_link', t):
        loc = self.localizationOK.query(t)
        if loc == None:
          if len(self.localizationOK.q) > 0:
            print "No localizations for", t, "earliest", min(self.localizationOK.q), "latest", max(self.localizationOK.q)
        if loc != None:
          if loc == (True,True):
            if self.isVerbosity(1):
              print "PM adding frame"
            picture.pose = getpose(transformer, 'map', 'stereo_link', t)
            if not(cam in self.cameras):
              self.cameras.append(cam)
            ci = self.cameras.index(cam)
            self.pics.append((ci, picture))
            self.vt.add(None, picture.descriptors)
            if 1 <= self.verbose:
              print "Added image to picture map"
          del self.observations[(t,(cam,picture,iskey))]

    for (t,(cam,picture,iskey)) in self.observations:
      # If localization is failing, look up the frame and return its pose
      if not iskey and transformer.canTransform('stereo_link', 'base_link', t):
        loc = self.localizationOK.query(t)
        if loc != None:
          if (loc[0] == False or loc[1] == False):
            kp = picture.keypoints
            descriptors = picture.descriptors

            best = self.find(descriptors)
            match_cam = self.cameras[self.pics[best][0]]
            match_pic = self.pics[best][1]

            pairs = self.ds.match0(match_pic.keypoints, match_pic.descriptors, kp, descriptors)
            pairs = [(b,a) for (a,b,d) in pairs]
            (inl,R,S) = self.pe.estimateC(match_cam, match_pic.keypoints, cam, kp, pairs)
            if inl > 100:
              r33 = numpy.mat(numpy.array(R).reshape(3,3))
              rel_pose = Pose(r33, numpy.array(S))
              cam90 = Pose(numpy.array([[ 0, 0, 1 ], [ -1, 0, 0 ], [ 0, -1, 0 ]]), numpy.array([0, 0, 0]))
              rel_pose = cam90 * rel_pose * ~cam90

              # Figure out camera's pose in map

              m2s = match_pic.pose
              cam_in_map = (m2s * rel_pose)
              base_in_map = cam_in_map * getpose(transformer, 'stereo_link', 'base_link', t)
              r = (t, base_in_map)
            else:
              r = None
            del self.observations[(t,(cam,picture,iskey))]
            return r
    return None

  def __getstate__(self):
    return (self.cameras, self.pics)

  def __setstate__(self, st):
    self.__init__()
    (self.cameras, self.pics) = st
    for p in self.pics:
      self.vt.add(None, p[1].descriptors)

