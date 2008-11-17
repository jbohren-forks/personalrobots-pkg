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

class VODemo:
  vo = None
  frame = 0

  def __init__(self, mode, library):
    self.mode = mode
    self.library = library

    rospy.TopicSub('/videre/images', ImageArray, self.display_array)
    rospy.TopicSub('/videre/cal_params', String, self.display_params)
    rospy.TopicSub('/vo/tmo', Pose44, self.handle_corrections)

    self.viz_pub = rospy.Publisher("/overlay", Lines)
    self.vo_key_pub = rospy.Publisher("/vo/key", Frame)
    self.Tmo = Pose()

    self.mymarker1 = Marker(10)
    self.mymarkertrail = [ Marker(11 + i) for i in range(10) ]
    self.trail = []

    self.vis = Vis()

  def display_params(self, iar):
    if not self.vo:
      cam = camera.VidereCamera(iar.data)
      print "cam.params", cam.params
      self.vo = VisualOdometer(cam)
      self.started = None
      if self.mode == 'learn':
        self.library = set()
      self.previous_keyframe = None
      self.know_state = 'lost'
      self.best_show_pose = None
      self.mymarker1.floor()

  def handle_corrections(self, corrmsg):
    print "GOT CORRECTION AT", time.time()
    Tmo_pose = Pose()
    Tmo_pose.fromlist(corrmsg.v)
    self.Tmo = Tmo_pose
    self.know_state = 'corrected'

  def display_array(self, iar):
    diag = ""
    af = None
    if self.vo:
      if not self.started:
        self.started = time.time()
      imgR = imgAdapted(iar.images[0])
      imgL = imgAdapted(iar.images[1])
      af = SparseStereoFrame(imgL, imgR)

      if 1:
        if self.mode == 'play':
          pose = self.vo.handle_frame(af)
        if self.mode == 'learn':
          pose = self.vo.handle_frame(af)
          if (af.id != 0) and (self.vo.inl < 80):
            print "*** LOST TRACK ***"
            #sys.exit(1)
          self.library.add(self.vo.keyframe)
        else:
          #diag = "best match %d from %d in library" % (max(probes)[0], len(self.library))
          pass
        diag = "%d/%d inliers, moved %.1f library size %d" % (self.vo.inl, len(af.kp), pose.distance(), len(self.library))

        if self.mode == 'play':
          kf = self.vo.keyframe
          if kf != self.previous_keyframe:
            f = Frame()
            f.id = kf.id
            f.pose = Pose44(kf.pose.tolist())
            f.keypoints = [ Keypoint(x,y,d) for (x,y,d) in kf.kp ]
            f.descriptors = [ Descriptor(d) for d in kf.descriptors ]
            print "ASKING FOR MATCH AT", time.time()
            self.vo_key_pub.publish(f)
            self.previous_keyframe = kf
            if kf.inl < 50 or self.vo.inl < 50:
              self.know_state = 'lost'
            else:
              self.know_state = { 'lost':'lost', 'uncertain':'uncertain', 'corrected':'uncertain' }[self.know_state]

        result_pose = af.pose
        if self.mode == 'learn':
          self.mymarker1.frompose(af.pose, self.vo.cam, (255,255,255))
        else:
          if self.best_show_pose and self.know_state == 'lost':
            inmap = self.best_show_pose
          else:
            Top = af.pose
            Tmo = self.Tmo
            inmap = Tmo * Top
            if self.know_state != 'lost':
              self.best_show_pose = inmap
          if self.know_state != 'lost' or not self.best_show_pose:
            color = { 'lost' : (255,0,0), 'uncertain' : (127,127,0), 'corrected' : (0,255,0) }[self.know_state]
            self.mymarker1.frompose(inmap, self.vo.cam, color)
            if 0:
              self.trail.append(inmap)
              self.trail = self.trail[-10:]
              for i,p in enumerate(self.trail):
                self.mymarkertrail[i].frompose(p, color)
        #print af.diff_pose.xform(0,0,0), af.pose.xform(0,0,0)

        if self.frame > 5 and ((self.frame % 10) == 0):
          inliers = self.vo.pe.inliers()
          pts = [(1,int(x0),int(y0)) for ((x0,y0,d0), (x1,y1,d1)) in inliers]
          self.vis.show(iar.images[1].data, pts )

        if False and self.vo.pairs != []:
          ls = Lines()
          inliers = self.vo.pe.inliers()
          lr = "left_rectified"
          ls.lines = [ Line(lr, 0,255,0,x0,y0-2,x0,y0+2) for ((x0,y0,d0), (x1,y1,d1)) in inliers]
          ls.lines += [ Line(lr, 0,255,0,x0-2,y0,x0+2,y0) for ((x0,y0,d0), (x1,y1,d1)) in inliers]
          rr = "right_rectified"
          #ls.lines += [ Line(rr, 0,255,0,x0-d0,y0-2,x0-d0,y0+2) for ((x0,y0,d0), (x1,y1,d1)) in inliers]
          #ls.lines += [ Line(rr, 0,255,0,x0-2-d0,y0,x0+2-d0,y0) for ((x0,y0,d0), (x1,y1,d1)) in inliers]
          self.viz_pub.publish(ls)

        if (self.frame % 30) == 0:
          took = time.time() - self.started
          print "%4d: %5.1f [%f fps]" % (self.frame, took, self.frame / took), diag
        self.frame += 1

    #print "got message", len(iar.images)
    #print iar.images[0].width
    if SEE:
      right = ut.ros2cv(iar.images[0])
      left  = ut.ros2cv(iar.images[1])
      hg.cvShowImage('channel L', left)
      hg.cvShowImage('channel R', right)
      hg.cvWaitKey(5)

  def dump(self):
    print
    print self.vo.name()
    self.vo.summarize_timers()
    if self.mode == 'learn':
      print "DUMPING LIBRARY", len(self.library)
      f = open("library.pickle", "w")
      pickle.dump(self.vo.cam, f)
      db = [(af.id, af.pose, af.kp, af.descriptors) for af in self.library]
      pickle.dump(db, f)
      f.close()
      print "DONE"

def main(args):

  time.sleep(3)
  optlist, args = getopt.getopt(args[1:], 'l')
  if ('-l', '') in optlist:
    mode = 'learn'
  else:
    mode = 'play'

  if mode == 'play':
    f = open("pruned.pickle", "r")
    cam = pickle.load(f)
    db = pickle.load(f)
    f.close()

    class LibraryFrame:
      def __init__(self, id, pose, kp, desc):
        self.id = id
        self.pose = pose
        self.kp = kp
        self.descriptors = desc

    library = set()
    for (id, pose, kp, desc) in db:
      lf = LibraryFrame(id, pose, kp, desc)
      library.add(lf)
  else:
    library = None

  if SEE:
    hg.cvNamedWindow('channel L', 1)
    hg.cvNamedWindow('channel R', 1)

  vod = VODemo(mode, library)

  rospy.ready('camview_py')
  rospy.spin()
  vod.dump()

if __name__ == '__main__':
  main(sys.argv)
