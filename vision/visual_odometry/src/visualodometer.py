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
#

## Main module for visual odometry.
# authors: jamesb

import vop
import votools as VO
from timer import Timer

import Image
from math import *
import numpy
import numpy.linalg

scratch = " " * (640 * 480)

import transformations

class Pose:
  def __init__(self, R=None, S=None):
    if R == None:
      R = numpy.identity(3)
    if S == None:
      S = ( 0, 0, 0)
    self.M = numpy.mat(numpy.vstack((numpy.hstack((R, numpy.array(S).reshape((3,1)))), numpy.array([0, 0, 0, 1]))))

  def fromlist(self, L):
    a = numpy.array(L)
    a.shape = 4,4
    self.M = numpy.mat(a)

  def tolist(self):
    return sum(self.M.tolist(), [])

  def invert(self):
    r = Pose()
    r.M = numpy.linalg.inv(self.M)
    return r

  def __mul__(self, other):
    r = Pose()
    r.M = self.M * other.M
    return r

  def __invert__(self):
    r = Pose()
    #r.M = self.M.T
    r.M = numpy.linalg.inv(self.M)
    return r

  def concatenate(self, P):
    r = Pose()
    r.M = numpy.dot(self.M, P.M)
    return r

  def xform(self, x, y, z):
    x,y,z,w = tuple(numpy.array(numpy.dot(self.M, numpy.array([x, y, z, 1]) ))[0])
    x /= w
    y /= w
    z /= w
    return (x,y,z)

  def quaternion(self):
    return transformations.quaternion_from_rotation_matrix(self.M)

  def euler(self):
    return transformations.euler_from_rotation_matrix(self.M)

  def compare(self, other):
    p0 = self.xform(0, 0, 0)
    p1 = other.xform(0, 0, 0)
    eu0 = self.euler()
    eu1 = other.euler()
    d = p0 - p1
    eud = [abs(eu0[i] - eu1[i]) for i in [0,1,2]]
    return (sqrt(numpy.dot(d, d.conj())), eud[0], eud[1], eud[2])

  def distance(self):
    x,y,z = self.xform(0,0,0)
    return sqrt(x * x + y * y + z * z)

import fast

class FeatureDetector:

  def name(self):
    return self.__class__.__name__

  def __init__(self):
    self.thresh = self.default_thresh
    self.cold = True

  def detect(self, frame, target_points):

    features = self.get_features(frame, target_points)
    if len(features) < (target_points * 0.5) or len(features) > (target_points * 2.0):
        (lo,hi) = self.threshrange
        for i in range(7):
          self.thresh = 0.5 * (lo + hi)
          features = self.get_features(frame, target_points)
          if len(features) < target_points:
            hi = self.thresh
          if len(features) > target_points:
            lo = self.thresh

    # Try to be a bit adaptive for next time
    if len(features) > (target_points * 1.1):
        self.thresh *= 1.05
    if len(features) < (target_points * 0.9):
        self.thresh *= 0.95
    return features

class FeatureDetectorFast(FeatureDetector):

  default_thresh = 10
  threshrange = (5,127)

  def get_features(self, frame, target_points):
    assert len(frame.rawdata) == (frame.size[0] * frame.size[1])
    return fast.fast(frame.rawdata, frame.size[0], frame.size[1], int(self.thresh), 40)

class FeatureDetector4x4:

  def __init__(self, fd):
    self.fds = [ fd() for i in range(16) ]

  def name(self):
    return "4x4 " + self.fds[0].__class__.__name__

  def detect(self, frame, target_points):
    w,h = frame.size
    master = Image.fromstring("L", frame.size, frame.rawdata)
    allpts = []
    for x in range(4):
      for y in range(4):
        xleft = x * (w/4)
        ytop = y * (h/4)
        subimage = master.crop((xleft, ytop, xleft + (w/4), ytop + (h/4)))
        assert subimage.size == ((w/4), (h/4))

        class FrameAdapter:
          def __init__(self, im):
            self.size = im.size
            self.rawdata = im.tostring()

        subpts = self.fds[4 * x + y].detect(FrameAdapter(subimage), target_points / 16)
        allpts += [(xleft + xp, ytop + yp) for (xp,yp) in subpts]
    return allpts

class FeatureDetectorHarris(FeatureDetector):

  default_thresh = 1e-3
  threshrange = (5,127)

  def get_features(self, frame, target_points):
    return VO.harris(frame.rawdata, frame.size[0], frame.size[1], int(target_points * 1.2), self.thresh, 2.0)

import starfeature

class FeatureDetectorStar(FeatureDetector):

  default_thresh = 30.0
  threshrange = (1,64)
  line_thresh = 10.0

  def get_features(self, frame, target_points):
    sd = starfeature.star_detector(frame.size[0], frame.size[1], 5, self.thresh, self.line_thresh)
    return [ (x,y) for (x,y,s,r) in sd.detect(frame.rawdata) ]

class DescriptorScheme:

  def name(self):
    return self.__class__.__name__

  def match(self, af0, af1):
    if af0.kp == [] or af1.kp == []:
      return []
    Xs = vop.array([k[0] for k in af1.kp])
    Ys = vop.array([k[1] for k in af1.kp])
    pairs = []
    for (i,(ki,di)) in enumerate(zip(af0.kp,af0.descriptors)):
      # hits = (Numeric.logical_and(Numeric.absolute(NXs - ki[0]) < 64, Numeric.absolute(NYs - ki[1]) < 32)).astype(Numeric.UnsignedInt8).tostring()
      predX = (abs(Xs - ki[0]) < 64)
      predY = (abs(Ys - ki[1]) < 32)
      hits = vop.where(predX & predY, 1, 0).tostring()
      best = self.search(di, af1, hits)
      if best != None:
        pairs.append((i, best[0], best[1]))
    return pairs

class DescriptorSchemeSAD(DescriptorScheme):

  def collect(self, frame):
    lgrad = " " * (frame.size[0] * frame.size[1])
    VO.ost_do_prefilter_norm(frame.rawdata, lgrad, frame.size[0], frame.size[1], 31, scratch)
    frame.descriptors = [ VO.grab_16x16(lgrad, frame.size[0], p[0]-7, p[1]-7) for p in frame.kp ]

  def search(self, di, af1, hits):
      i = VO.sad_search(di, af1.descriptors, hits)
      if i == None:
        return None
      else:
        return (i, 0)

import calonder

class DescriptorSchemeCalonder(DescriptorScheme):
  def __init__(self):
    self.cl = calonder.classifier()
    #self.cl.setThreshold(0.0)
    self.cl.read('/u/prdata/calonder_trees/land50.trees')
    self.ma = calonder.BruteForceMatcher()

  def collect(self, frame):
    frame.descriptors = []
    im = Image.fromstring("L", frame.size, frame.rawdata)
    frame.matcher = calonder.BruteForceMatcher()
    for (x,y,d) in frame.kp:
      patch = im.crop((x-16,y-16,x+16,y+16))
      sig = self.cl.getSparseSignature(patch.tostring(), patch.size[0], patch.size[1])
      frame.descriptors.append(sig)
      frame.matcher.addSignature(sig)

  def search(self, di, af1, hits):
    match = af1.matcher.findMatch(di, hits)
    return match



class VisualOdometer:

  def __init__(self, cam, **kwargs):
    self.cam = cam
    self.timer = {}
    for t in ['feature', 'disparity', 'descriptor_collection', 'temporal_match', 'solve' ]:
      self.timer[t] = Timer()
    self.pe = VO.pose_estimator(*self.cam.params)
    self.prev_frame = None
    self.pose = Pose()
    self.inl = 0
    self.outl = 0
    self.num_frames = 0
    self.keyframe = None
    self.log_keyframes = []
    self.pairs = []

    self.angle_keypoint_thresh = kwargs.get('angle_keypoint_thresh', 0.05)
    self.inlier_thresh = kwargs.get('inlier_thresh', 175)
    self.feature_detector = kwargs.get('feature_detector', FeatureDetectorFast())
    self.descriptor_scheme = kwargs.get('descriptor_scheme', DescriptorSchemeSAD())

  def name(self):
    return "VisualOdometer< %s %s>" % (self.feature_detector.name(), self.descriptor_scheme.name())

  def reset_timers(self):
    for n,t in self.timer.items():
      t.reset();

  def average_time_per_frame(self):
    niter = self.num_frames
    return 1e3 * sum([t.sum for t in self.timer.values()]) / niter

  def summarize_timers(self):
    niter = self.num_frames
    print niter, "frames"
    for n,t in self.timer.items():
      print "%-20s %fms" % (n, 1e3 * t.sum / niter)
    print "%-20s %fms" % ("TOTAL", self.average_time_per_frame())

  targetkp = 400

  def find_keypoints(self, frame):
    self.timer['feature'].start()
    frame.kp2d = self.feature_detector.detect(frame, self.targetkp)
    self.timer['feature'].stop()

  def find_disparities(self, frame):
    self.timer['disparity'].start()
    disparities = [frame.lookup_disparity(x,y) for (x,y) in frame.kp2d]
    frame.kp = [ (x,y,z) for ((x,y),z) in zip(frame.kp2d, disparities) if z]
    self.timer['disparity'].stop()

  lgrad = " " * (640 * 480)
  def collect_descriptors(self, frame):
    self.timer['descriptor_collection'].start()
    self.descriptor_scheme.collect(frame)
    self.timer['descriptor_collection'].stop()

  def temporal_match(self, af0, af1, want_distances = False):
    """ Match features between two frames.  Returns a list of pairs of indices into the features in the two frames, and optionally a distance value for each pair, if want_distances in True.  """
    self.timer['temporal_match'].start()
    pairs = self.descriptor_scheme.match(af0, af1)
    if not want_distances:
      pairs = [(a,b) for (a,b,d) in pairs]
    self.timer['temporal_match'].stop()
    return pairs

  def solve(self, k0, k1, pairs, polish = True):
    self.timer['solve'].start()
    if pairs != []:
      #r = self.pe.estimate(k0, k1, pairs)
      r = self.pe.estimate(k1, k0, [ (b,a) for (a,b) in pairs ], polish)
    else:
      r = None
    self.timer['solve'].stop()

    return r

  def mkpose(self, rot, shift):
    r33 = numpy.mat(numpy.array(rot).reshape(3,3))
    return Pose(r33, numpy.array(shift))

    #pr = Pose(r33, numpy.array([0,0,0]))
    #ps = Pose(numpy.mat([[1,0,0],[0,1,0],[0,0,1]]), numpy.array(shift))
    #return pr * ps

  def proximity(self, f0, f1):
    """Given frames f0, f1, returns (inliers, pose) where pose is the transform that maps f1's frame int f0's frame.)"""
    self.num_frames += 1
    pairs = self.temporal_match(f0, f1)
    if len(pairs) > 10:
      solution = self.solve(f0.kp, f1.kp, pairs, False)
      (inl, rot, shift) = solution
      pose = self.mkpose(rot, shift)
      return (inl, pose)
    else:
      return (0, None)

  def handle_frame(self, frame):
    self.find_keypoints(frame)
    self.find_disparities(frame)
    self.collect_descriptors(frame)
    frame.id = self.num_frames
    return self.handle_frame_0(frame)

  def handle_frame_0(self, frame):
    if self.prev_frame:
      # If the key->current is good, use it
      # Otherwise, prev frame becomes the new key

      ref = self.keyframe
      self.pairs = self.temporal_match(ref, frame)
      solution = self.solve(ref.kp, frame.kp, self.pairs)
      if solution:
        (inl, rot, shift) = solution
        self.inl = inl
        self.outl = len(self.pairs) - inl
        diff_pose = self.mkpose(rot, shift)
      else:
        self.inl = 0
        self.outl = 0
        diff_pose = Pose()
      frame.diff_pose = diff_pose
      is_far = self.inl < self.inlier_thresh
      if (self.keyframe != self.prev_frame) and is_far:
        self.keyframe = self.prev_frame
        self.log_keyframes.append(self.keyframe.id)
        return self.handle_frame_0(frame)
      Tok = ref.pose
      Tkp = diff_pose
      Top = Tok * Tkp
      frame.pose = Top
      frame.inl = self.inl
    else:
      frame.pose = Pose()
      self.keyframe = frame
      self.log_keyframes.append(self.keyframe.id)
      frame.inl = 999

    self.pose = frame.pose
    self.prev_frame = frame

    self.num_frames += 1

    return self.pose
