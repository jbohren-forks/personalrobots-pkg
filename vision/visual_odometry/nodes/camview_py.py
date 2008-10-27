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

from math import *

from std_msgs.msg import Image, ImageArray, String
import rospy
import pyrob.util as ut
import opencv.highgui as hg
import visual_odometry as VO
from stereo import DenseStereoFrame, SparseStereoFrame
from visualodometer import VisualOdometer

import PIL.Image
import PIL.ImageDraw
import cPickle

SEE = 0

class imgAdapted:
  def __init__(self, i):
    self.i = i
    self.size = (i.width, i.height)
  def tostring(self):
    return self.i.data

def circle(im, x, y, r, color):
    draw = PIL.ImageDraw.Draw(im)
    box = [ int(i) for i in [ x - r, y - r, x + r, y + r ]]
    draw.pieslice(box, 0, 360, fill = color)

class VODemo:
  vo = None
  frame = 0

  def display_params(self, iar):
    if not self.vo:
      matrix = []         # Matrix will be in row,column order
      section = ""
      in_proj = 0
      for l in iar.data.split('\n'):
        if len(l) > 0 and l[0] == '[':
          section = l.strip('[]')
        ws = l.split()
        if ws != []:
          if section == "right camera" and ws[0].isalpha():
            in_proj = (ws[0] == 'proj')
          elif in_proj:
            matrix.append([ float(s) for s in l.split() ])
      Fx = matrix[0][0]
      Fy = matrix[1][1]
      Cx = matrix[0][2]
      Cy = matrix[1][2]
      Tx = -matrix[0][3] / Fx
      self.params = (Fx, Fy, Tx, Cx, Cx, Cy)
      self.vo = VisualOdometer(self.params)
      self.prev_af = None
      self.started = time.time()

  def display_array(self, iar):
    diag = ""
    af = None
    if self.vo:
      if 0:
        time.sleep(0.028)
      else:
        imgR = imgAdapted(iar.images[0])
        imgL = imgAdapted(iar.images[1])
        af = SparseStereoFrame(imgL, imgR)
        self.vo.find_keypoints(af)
        self.vo.find_disparities(af)
        self.vo.collect_descriptors(af)
        if 0:
          iL = PIL.Image.fromstring("L", imgL.size, imgL.tostring())
          #for (x,y) in af.kp2d:
          #  circle(iL, x, y, 2, 255)
          iL.save("out%04d.tiff" % self.frame)
        if self.prev_af:
          pairs = self.vo.temporal_match(self.prev_af, af)
          pose = self.vo.solve(self.prev_af.kp, af.kp, pairs)
          if pose and (pose[0] > 10):
            (inliers, rot, shift) = pose
            diag = "%d inliers, moved %.1f" % (inliers, sqrt(shift[0] * shift[0] + shift[1] * shift[1] + shift[2] * shift[2]))
          else:
            diag = "FAILED TO FIND"
      if (self.frame % 1) == 0:
        print "[%f fps]" % (self.frame / (time.time() - self.started)), diag
      if af:
        self.prev_af = af
      self.frame += 1
      if self.frame == 1000:
        self.vo.summarize_timers(self.frame)
        sys.exit(1)

    #print "got message", len(iar.images)
    #print iar.images[0].width
    if SEE:
      right = ut.ros2cv(iar.images[0])
      left  = ut.ros2cv(iar.images[1])
      hg.cvShowImage('channel L', left)
      hg.cvShowImage('channel R', right)
      hg.cvWaitKey(5)

def display_single(im):
    imcv = ut.ros2cv(im)
    hg.cvShowImage('channel 1', imcv)
    hg.cvWaitKey(5)

def main():
    vod = VODemo()
    if SEE:
      hg.cvNamedWindow('channel L', 1)
      hg.cvNamedWindow('channel R', 1)
    rospy.TopicSub('/videre/images', ImageArray, vod.display_array)
    rospy.TopicSub('/videre/cal_params', String, vod.display_params)
    rospy.TopicSub('image', Image, display_single)
    rospy.ready('camview_py')
    rospy.spin()

if __name__ == '__main__':
    import cProfile
    cProfile.run('main()', 'mainprof')
