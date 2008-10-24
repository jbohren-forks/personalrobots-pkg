import vop
import visual_odometry as VO
from timer import Timer

from math import *
import numpy

class Pose:
  def __init__(self, R=None, S=None):
    if R != None:
      self.M = numpy.mat(numpy.vstack((numpy.hstack((R, numpy.array(S).reshape((3,1)))), numpy.array([0, 0, 0, 1]))))
    else:
      self.M = numpy.mat(numpy.identity(4))

  def concatenate(self, P):
    r = Pose()
    r.M = numpy.dot(self.M, P.M)
    return r

  def xform(self, x, y, z):
    return numpy.array(numpy.dot(self.M, numpy.array([x, y, z, 1]) ).T[0])

  def quaternion(self):
    return transform.quaternion_from_rotation_matrix(self.M)

  def euler(self):
    return transform.euler_from_rotation_matrix(self.M)

  def compare(self, other):
    p0 = self.xform(0, 0, 0)
    p1 = other.xform(0, 0, 0)
    eu0 = self.euler()
    eu1 = other.euler()
    d = p0 - p1
    eud = [abs(eu0[i] - eu1[i]) for i in [0,1,2]]
    return (sqrt(numpy.dot(d, d.conj())), eud[0], eud[1], eud[2])

  def distance(self):
    d = numpy.array(numpy.dot(self.M, numpy.array([0, 0, 0, 1]) ).T[0])
    return sqrt(numpy.dot(d,d.conj()))

import fast

class FeatureDetector:

  def __init__(self):
    self.thresh = self.default_thresh
    self.cold = True

  def detect(self, frame, target_points):
    features = self.get_features(frame, target_points)
    if len(features) > (target_points * 1.1):
        self.thresh *= 1.05
    if len(features) < (target_points * 0.9):
        self.thresh *= 0.95
    return features

class FeatureDetectorFast(FeatureDetector):

  default_thresh = 10

  def get_features(self, frame, target_points):
    return fast.fast(frame.rawdata, frame.size[0], frame.size[1], int(self.thresh), 40)

class FeatureDetectorHarris(FeatureDetector):

  default_thresh = 1e-3

  def get_features(self, frame, target_points):
    return VO.harris(frame.rawdata, frame.size[0], frame.size[1], int(target_points * 1.2), self.thresh, 2.0)

import starfeature

class FeatureDetectorStar(FeatureDetector):

  default_thresh = 30.0
  line_thresh = 10.0

  def get_features(self, frame, target_points):
    sd = starfeature.star_detector(frame.size[0], frame.size[1], 5, self.thresh, self.line_thresh)
    return [ (x,y) for (x,y,s,r) in sd.detect(frame.rawdata) ]

scratch = " " * (640 * 480)

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
    self.num_frames = 0
    self.keyframe = None
    self.log_keyframes = []

    self.angle_keypoint_thresh = kwargs.get('angle_keypoint_thresh', 0.05)
    self.inlier_thresh = kwargs.get('inlier_thresh', 175)
    self.feature_detector = kwargs.get('feature_detector', FeatureDetectorFast)()

  def reset_timers(self):
    for n,t in self.timer.items():
      t.reset();

  def summarize_timers(self):
    niter = self.num_frames
    for n,t in self.timer.items():
      print "%-20s %fms" % (n, 1e3 * t.sum / niter)
    print "%-20s %fms" % ("TOTAL", 1e3 * sum([t.sum for t in self.timer.values()]) / niter)

  targetkp = 400
  def find_keypoints(self, frame):
    self.timer['feature'].start()
    frame.kp2d = self.feature_detector.detect(frame, self.targetkp)
    self.timer['feature'].stop()

  def find_disparities(self, frame):
    self.timer['disparity'].start()
    disparities = [frame.lookup_disparity(x,y) for (x,y) in frame.kp2d]
    frame.kp = [ (x,y,z) for ((x,y),z) in zip(frame.kp2d, disparities) if z != None ]
    self.timer['disparity'].stop()

  lgrad = " " * (640 * 480)
  def collect_descriptors(self, frame):
    self.timer['descriptor_collection'].start()
    VO.ost_do_prefilter_norm(frame.rawdata, self.lgrad, frame.size[0], frame.size[1], 31, scratch)
    frame.descriptors = [ VO.grab_16x16(self.lgrad, frame.size[0], p[0]-7, p[1]-7) for p in frame.kp ]
    self.timer['descriptor_collection'].stop()

  def temporal_match(self, af0, af1):
    self.timer['temporal_match'].start()
    Xs = vop.array([k[0] for k in af1.kp])
    Ys = vop.array([k[1] for k in af1.kp])
    pairs = []
    for (i,(ki,di)) in enumerate(zip(af0.kp,af0.descriptors)):
      # hits = (Numeric.logical_and(Numeric.absolute(NXs - ki[0]) < 64, Numeric.absolute(NYs - ki[1]) < 32)).astype(Numeric.UnsignedInt8).tostring()
      predX = (abs(Xs - ki[0]) < 64)
      predY = (abs(Ys - ki[1]) < 32)
      hits = vop.where(predX & predY, 1, 0).tostring()
      best = VO.sad_search(di, af1.descriptors, hits)
      if best:
        pairs.append((i, best))
    self.timer['temporal_match'].stop()
    return pairs

  def solve(self, k0, k1, pairs):
    self.timer['solve'].start()
    if pairs != []:
      #r = self.pe.estimate(k0, k1, pairs)
      r = self.pe.estimate(k1, k0, [ (b,a) for (a,b) in pairs ])
    else:
      r = None
    self.timer['solve'].stop()

    return r

  def handle_frame(self, frame):
    self.find_keypoints(frame)
    self.find_disparities(frame)
    self.collect_descriptors(frame)
    frame.id = self.num_frames
    if self.prev_frame:
      # If the key->current is good, use it
      # Otherwise, prev frame becomes the new key

      ref = self.keyframe
      self.pairs = self.temporal_match(ref, frame)
      solution = self.solve(ref.kp, frame.kp, self.pairs)
      (inl, rot, shift) = solution
      self.inl = inl
      r33 = numpy.mat(numpy.array(rot).reshape(3,3))
      diff_pose = Pose(r33, numpy.array(shift))
      is_far = inl < self.inlier_thresh
      if (self.keyframe != self.prev_frame) and is_far:
        self.keyframe = self.prev_frame
        self.log_keyframes.append(self.keyframe.id)
        #print "new key", self.keyframe.id
        return self.handle_frame(frame)
      frame.pose = ref.pose.concatenate(diff_pose)
    else:
      frame.pose = Pose()
      self.keyframe = frame
      self.log_keyframes.append(self.keyframe.id)

    self.pose = frame.pose
    self.prev_frame = frame
    
    diff = self.pose.compare(self.keyframe.pose)
    if 0 and (max(diff[1:]) > self.angle_keypoint_thresh):
      self.keyframe = frame
      print "KEYFRAME", frame.id

    self.num_frames += 1

    return self.pose
