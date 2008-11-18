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
from visualodometer import VisualOdometer, Pose
import camera
from marker import Marker

import pickle
import random
import time

from Queue import Queue

class LibraryFrame:
  def __init__(self, id, pose, kp, desc):
    self.id = id
    self.pose = pose
    self.kp = kp
    self.descriptors = desc

class Corrector:
  vo = None
  frame = 0

  def __init__(self, vo, library):
    self.vo = vo
    self.library = library

    rospy.TopicSub('/vo/key', Frame, self.incoming_frame)
    self.pub_tmo = rospy.Publisher("/vo/tmo", Pose44)
    self.frameq = Queue()

    self.mark0 = Marker(1)
    self.mark1 = Marker(2)

  def incoming_frame(self, af):
    print "CORR: GOT INCOMING FRAME AT", time.time()
    self.frameq.put(af)

  def workloop(self):
    workframe = None

    while not rospy.is_shutdown():
      while self.frameq.empty() and not rospy.is_shutdown():
        time.sleep(0.1)
      while not self.frameq.empty() and not rospy.is_shutdown():
        afmsg = self.frameq.get()
        print "CORR: NEW WORK", "id", afmsg.id
        p = Pose()
        p.fromlist(afmsg.pose.v)
        workframe = LibraryFrame(afmsg.id, p, [ (p.x,p.y,p.d) for p in afmsg.keypoints], [d.v for d in afmsg.descriptors])
        assert len(workframe.descriptors) == len(workframe.kp)
        for d in workframe.descriptors:
          assert len(d) == 256

      if workframe:
        tosearch = list(self.library)
        best = 100
        best_frame = None
        best_diff = None
        for essay in tosearch:
          prox,diffpose = self.vo.proximity(essay, workframe)
          print "at", time.time(), "essay.id", essay.id, "has prox", prox
          if essay.id == workframe.id:
            print len(essay.kp), len(workframe.kp)

          if prox > best:
            best = prox
            best_frame = essay
            best_diff = diffpose

        if best_frame:
          print "CORR: sending winner", best_frame.id, "at", time.time()

          Top = workframe.pose
          Tmk = best_frame.pose
          Tkp = best_diff
          Tmp = Tmk * Tkp
          Tmo = Tmp * ~Top
          for mat in [ "Top", "Tmk", "Tkp", "Tmp", "Tmo", "Tmo*Top" ]:
            print mat
            print eval(mat).M
            print
          print "corrector has pose Tmp:"
          print (Tmo * Top).M
          if 1:
            a = Tmp.tolist()
            b = (Tmo * Top).tolist()
            for ai,bi in zip(a,b):
              if abs(ai - bi) > 0.001:
                print "FAIL"
                for mat in [ "Top", "Tmp", "Tmo", "Tmo*Top" ]:
                  print mat
                  print eval(mat).M
                assert 0

          self.pub_tmo.publish(Pose44(Tmo.tolist()))
          if 0:
            Tpo = workframe.pose
            Tpm = best_diff * best_frame.pose
            Tp = ~Tpo
            Tom = Top * Tpm
            print "Tpo"
            print Tpo.M
            print "Tpm"
            print Tpm.M
            print "Computed Tom"
            print Tom.M
            print "Keyframe in map"
            print (Tpo * Tom).M
            self.pub_tom.publish(Pose44(Tom.tolist()))
          #p = workframe.pose.invert().concatenate(best_frame.pose).concatenate(best_diff)
        else:
          print "CORR: did not find acceptable winner"
      else:
        time.sleep(0.1)

def main(args):

  f = open("pruned.pickle", "r")
  cam = pickle.load(f)
  db = pickle.load(f)
  f.close()

  print cam.params
  vo = VisualOdometer(cam)

  library = set()
  for (id, pose, kp, desc) in db:
    lf = LibraryFrame(id, pose, kp, desc)
    library.add(lf)

  corr = Corrector(vo, library)

  rospy.ready('corrector')
  try:
    corr.workloop()
  except KeyboardInterrupt:
    print "shutting down"

if __name__ == '__main__':
  main(sys.argv)
