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
import visual_odometry.lowlevel as VOLO
from stereo_utils.timer import TimedClass

import os

import Image
from math import *
import numpy
import numpy.linalg

from tf import transformations

class Pose:
  """ A Pose is a convenience wrapper for a numpy 4x4 matrix """
  def __init__(self, R=None, S=None):
    if R == None:
      R = numpy.identity(3)
    if S == None:
      S = ( 0, 0, 0)
    self.M = numpy.mat(numpy.vstack((numpy.hstack((R, numpy.array(S).reshape((3,1)))), numpy.array([0, 0, 0, 1]))))

  def fromlist(self, L):
    """ Initialize the pose from the 16 element list *L*. """
    a = numpy.array(L)
    a.shape = 4,4
    self.M = numpy.mat(a)

  def tolist(self):
    """ Return the 16 element list for the pose."""
    return sum(self.M.tolist(), [])

  def __mul__(self, other):
    """Matrix multiplication"""
    r = Pose()
    r.M = self.M * other.M
    return r

  def __invert__(self):
    """Matrix inverse"""
    r = Pose()
    #r.M = self.M.T
    r.M = numpy.linalg.inv(self.M)
    return r

  def __repr__(self):
    return str(self.M)

  def xform(self, x, y, z):
    """ Transform point (*x*, *y*, *z*) by the matrix.  Returns triple (x', y', z'). """
    if 0:
      x,y,z,w = tuple(numpy.array(numpy.dot(self.M, numpy.array([x, y, z, 1]) ))[0])
      assert w == 1.0
      return (x,y,z)
    else:
      # More efficient implementation that plays well with vop for the the optimized PE
      nx = float(self.M[0,0]) * x + float(self.M[0,1]) * y + float(self.M[0,2]) * z + float(self.M[0,3])
      ny = float(self.M[1,0]) * x + float(self.M[1,1]) * y + float(self.M[1,2]) * z + float(self.M[1,3])
      nz = float(self.M[2,0]) * x + float(self.M[2,1]) * y + float(self.M[2,2]) * z + float(self.M[2,3])
      return (nx, ny, nz)

  def quaternion(self):
    """ Return the quaternion representation of the rotation of the pose matrix """
    return transformations.quaternion_from_rotation_matrix(self.M)

  def euler(self):
    """ Return the Euler representation of the rotation of the pose matrix """
    return transformations.euler_from_rotation_matrix(self.M)

  def compare(self, other):
    p0 = numpy.array(self.xform(0, 0, 0))
    p1 = numpy.array(other.xform(0, 0, 0))
    eu0 = self.euler()
    eu1 = other.euler()
    d = p0 - p1
    eud = [abs(eu0[i] - eu1[i]) for i in [0,1,2]]
    return (sqrt(numpy.dot(d, d.conj())), eud[0], eud[1], eud[2])

  def qangle(self, other):
    (wx,wy,wz,ww) = self.quaternion()
    yaw1 = atan2(2*(wx*wy + ww*wz), wx*wx + ww*ww - wy*wy - wz*wz)
    (wx,wy,wz,ww) = other.quaternion()
    yaw2 = atan2(2*(wx*wy + ww*wz), wx*wx + ww*ww - wy*wy - wz*wz)
    return yaw2 - yaw1

  def angle(self, other):
    p0 = self.xform(0,0,0)
    d0 = self.xform(0,0,1)
    p1 = other.xform(0,0,0)
    d1 = other.xform(0,0,1)
    v0 = [ (b - a) for (a,b) in zip(p0,d0) ]
    v1 = [ (b - a) for (a,b) in zip(p1,d1) ]
    dot = sum([(a*b) for a,b in zip(v0,v1)])
    if dot >= 1.0:
      th = 0.0
    else:
      th = acos(dot)
    return th

  def further_than(self, other, pos_d, ori_d):
    p0 = self.xform(0,0,0)
    p1 = other.xform(0,0,0)
    th = self.qangle(other)
    d = sqrt(sum([(a - b)**2 for (a,b) in zip(p0, p1)]))
    return (d > pos_d) or (abs(th) > ori_d)

  def d(self, other):
    p0 = self.xform(0,0,0)
    p1 = other.xform(0,0,0)
    return sqrt(sum([(a - b)**2 for (a,b) in zip(p0, p1)]))

  def distance(self):
    x,y,z = self.xform(0,0,0)
    return sqrt(x * x + y * y + z * z)

  def assert_sane(self):
    rot = self.M[0:3,0:3]
    #assert numpy.alltrue(numpy.abs(((rot * rot.T) - numpy.identity(3))) < 1.0e-3)

def from_xyz_euler(xyz, euler):
  """ Return a :class:`Pose` from an xyz position and triple *euler*. """
  R = transformations.rotation_matrix_from_euler(euler[0], euler[1], euler[2], 'sxyz')
  return Pose(R[:3,:3], xyz)

########################################################################
uniq_track_id = 100

class Track:
  def __init__(self, p0, p0id, p1, p1id, p1pose, cam):
    global uniq_track_id
    self.p = [p0, p1]
    self.id = [p0id, p1id]
    self.lastpt = p1
    self.alive = True
    #print p0id,p0,p1id,p1,p1pose.xform(*cam.pix2cam(*p1)),uniq_track_id
    self.sba_track = VOLO.point_track(p0id, p0, p1id, p1, p1pose.xform(*cam.pix2cam(*p1)), uniq_track_id)
    self.uniq_track_id = uniq_track_id
    uniq_track_id += 1
  def kill(self):
    self.alive = False
  def extend(self, p1, p1id):
    self.p.append(p1)
    self.id.append(p1id)
    self.lastpt = p1
    self.sba_track.extend(p1id, p1)

import pe

class VisualOdometer(TimedClass):
  """
  VisualOdometer takes a series of views from a camera *cam*, and returns the relative pose of each view.
  The visual odometer reduces accumulated error by computing pose from a previous key frame, and only moving the key frame
  forwards when certain thresholds (below) are exceeded.
  Keyword arguments:

  position_keypoint_thresh
        Threshold distance; when current frame's distance to key frame
        exceeds this limit, key frame is moved forward.  Default value
        0.5.

  angle_keypoint_thresh
        Threshold angle (radians); when current frame's angle to key frame
        exceeds this limit, key frame is moved forward.  Default value
        pi / 18.

  inlier_thresh
        Threshold inlier count; when current frame's inlier count is
        below this value, key frame is moved forward.  Default value 175.

  scavenge
        Scavenger mode; first a first pass to obtain a pose estimate,
        then re-runs the frame's matchers using the pose estimate to
        restrict the search area.  Default value False.

  num_ransac_iters
        Number of iterations for the pose estimator's RANSAC.  Default value 100.

  """
  def __init__(self, cam, **kwargs):
    self.cam = cam
    TimedClass.__init__(self, ['temporal_match', 'solve'])

    #self.pe = VOLO.pose_estimator(*self.cam.params)
    self.pe = pe.PoseEstimator(*self.cam.params)

    self.prev_frame = None
    self.pose = Pose()
    self.inl = 0
    self.outl = 0
    self.num_frames  = 0
    self.tot_inliers = 0
    self.tot_matches = 0
    self.tot_points  = 0
    self.ext_frames = 1000000
    self.keyframe = None
    self.log_keyframes = []
    self.log_keyposes = []
    self.pairs = []
    self.tracks = set()
    self.all_tracks = set()
    self.posechain = []

    self.position_thresh = kwargs.get('position_keypoint_thresh', 0.5)
    self.angle_thresh = kwargs.get('angle_keypoint_thresh', (2 * pi) / (5. / 360))
    self.inlier_thresh = kwargs.get('inlier_thresh', 175)
    self.inlier_error_threshold = kwargs.get('inlier_error_threshold', 3.0)
    self.scavenge = kwargs.get('scavenge', False)
    self.sba = kwargs.get('sba', None)
    self.num_ransac_iters = kwargs.get('ransac_iters', 100)

    self.pe.setInlierErrorThreshold(self.inlier_error_threshold)
    self.pe.setNumRansacIterations(self.num_ransac_iters)

  def name(self):
    return "VisualOdometer (iet=%.1f sba=%s)" % (self.inlier_error_threshold, str(self.sba))

  def reset_timers(self):
    for n,t in self.timer.items():
      t.reset();

  def average_time_per_frame(self):
    niter = self.num_frames
    return 1e3 * sum([t.sum for t in self.timer.values()]) / niter

  def summarize_timers(self):
    TimedClass.summarize_timers(self)
    niter = self.num_frames
    print ""
    print "total number of keyframes/frames:", len(self.log_keyframes), "/", niter
    print "average number of points  per frame: ", self.tot_points/niter
    print "average number of matches per frame: ", self.tot_matches/niter
    print "average number of inliers per frame: ", self.tot_inliers/niter
    print ""
    if niter != 0:
      for n,t in self.timer.items():
        print "  %-20s %fms (%s)" % (n, 1e3 * t.sum / niter, t.summ())
      print "  %-20s %fms" % ("TOTAL", self.average_time_per_frame())

  def temporal_match(self, af0, af1, want_distances = False):
    """
    Match features between two frames.  Returns a list of pairs
    of indices into the features in the two frames, and optionally a
    distance value for each pair, if want_distances is True.
    """
    self.timer['temporal_match'].start()
    pairs = af0.match(af1)
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
    if not rot:
      return Pose()
    else:
      r33 = numpy.mat(numpy.array(rot).reshape(3,3))
      return Pose(r33, numpy.array(shift))

  def show_pairs(self, pairs, f0, f1):
    print "*** SHOWING PAIRS FOR FRAMES ", f0.id, f1.id, "***"
    print f0.id, "has", len(f0.kp), "keypoints"
    print f1.id, "has", len(f1.kp), "keypoints"
    print "There are", len(pairs), "pairs"
    import pylab
    pylab.imshow(numpy.fromstring(f0.lf.tostring(), numpy.uint8).reshape(480,640), cmap=pylab.cm.gray)
    pylab.scatter([x for (x,y,d) in f0.kp], [y for (x,y,d) in f0.kp], label = '%d kp' % f0.id, c = 'red')
    pylab.figure()
    pylab.imshow(numpy.fromstring(f1.lf.tostring(), numpy.uint8).reshape(480,640), cmap=pylab.cm.gray)
    pylab.scatter([x for (x,y,d) in f1.kp], [y for (x,y,d) in f1.kp], label = '%d kp' % f1.id, c = 'green')
    pylab.figure()
    for (a,b) in pairs:
      pylab.plot([ f0.kp[a][0], f1.kp[b][0] ], [ f0.kp[a][1], f1.kp[b][1] ])
    pylab.imshow(numpy.fromstring(f0.lf.tostring(), numpy.uint8).reshape(480,640), cmap=pylab.cm.gray)
    pylab.scatter([x for (x,y,d) in f0.kp], [y for (x,y,d) in f0.kp], label = '%d kp' % f0.id, c = 'red')
    pylab.scatter([x for (x,y,d) in f1.kp], [y for (x,y,d) in f1.kp], label = '%d kp' % f1.id, c = 'green')
    pylab.legend()
    pylab.show()

  def scavenger(self, diff_pose, af0, af1):
    Xs = vop.array([k[0] for k in af1.features()])
    Ys = vop.array([k[1] for k in af1.features()])
    pairs = []
    fwd_pose = ~diff_pose
    ds = af1.descriptor_scheme
    matcher = ds.desc2matcher(af1.descriptors())
    for (i,(ki,di)) in enumerate(zip(af0.features(),af0.descriptors())):
      (x,y,d) = self.cam.cam2pix(*fwd_pose.xform(*self.cam.pix2cam(*ki)))
      predX = (abs(Xs - x) < 4)
      predY = (abs(Ys - y) < 4)
      hits = vop.where(predX & predY, 1, 0).tostring()
      best = ds.search(di, matcher, hits)
      if best != None:
        pairs.append((i, best[0], best[1]))
    self.pairs = [(i0,i1) for (i0,i1,d) in pairs]
    if False:
      import pylab
      f0,f1 = af0,af1
      for (a,b) in self.pairs:
        pylab.plot([ f0.features()[a][0], f1.features()[b][0] ], [ f0.features()[a][1], f1.features()[b][1] ])
      pylab.imshow(numpy.fromstring(af0.lf.tostring(), numpy.uint8).reshape(480,640), cmap=pylab.cm.gray)
      pylab.scatter([x for (x,y,d) in f0.features()], [y for (x,y,d) in f0.features()], label = '%d kp' % f0.id, c = 'red')
      pylab.scatter([x for (x,y,d) in f1.features()], [y for (x,y,d) in f1.features()], label = '%d kp' % f1.id, c = 'green')
      pylab.legend()
      pylab.show()
    solution = self.solve(af0.features(), af1.features(), self.pairs)
    return solution

  def maintain_tracks(self, f0, f1):
    self.timer['tracks'].start()
    pairs = self.temporal_match(f0, f1)
    solution = self.solve(f0.kp, f1.kp, pairs)
    if solution and solution[0] > 5:
      (inl, rot, shift) = solution
      diff_pose = self.mkpose(rot, shift)
      if self.scavenge:
        (inl, rot, shift) = self.scavenger(diff_pose, f0, f1)
    # for points x in f0, pairmap[x] is the corresponding point in f1
    pairmap = dict([(p1,p0) for (p0,p1) in self.pe.inliers()])

    for t in self.tracks:
      if not (t.lastpt in pairmap):
        t.kill()

    if self.posechain != []:
      # Only keep tracks that have a recent frame
      (fix_sz, free_sz, niter) = self.sba
      nfr = fix_sz + free_sz
      print "posechain length", len(self.posechain)
      print "posechain ", (self.posechain)
      if len(self.posechain) > nfr:
        oldest_useful_frame = self.posechain[-nfr][0].id
      else:
        oldest_useful_frame = self.posechain[0][0].id
      #print "oldest_useful_frame", oldest_useful_frame

      def age(t):
        return max([ i for i in t.id if i < 1000000])
      self.tracks = set([ t for t in self.tracks if oldest_useful_frame <= age(t)])

    oldtails = set([t.lastpt for t in self.tracks])
    for t in self.tracks:
      if t.alive:
        t.extend(pairmap[t.lastpt], f1.id)

    #print len(self.tracks), [ len(t.p) for t in self.tracks ]
    for p0 in set(pairmap) - oldtails:
      p1 = pairmap[p0]
      newtrack = Track(p0, f0.id, p1, f1.id, f1.pose, self.cam)
      self.tracks.add(newtrack)
      self.all_tracks.add(newtrack)

    if 0:
      import pylab
      print "Inliers:", len(self.pe.inliers())

      f1in = [ a for (a,b) in self.pe.inliers()]
      pylab.scatter([x for (x,y,d) in f1in], [y for (x,y,d) in f1in])

      print "There are", len(self.tracks)
      for t in self.tracks:
        pylab.plot([x for (x,y,d) in t.p], [y for (x,y,d) in t.p])
      pylab.xlim((0, 640))
      pylab.ylim((0, 480))
      pylab.show()
    
    # Run through all tracks.  If two tracks share an endpoint, delete the shorter track
    by_lastpt = [e[1] for e in sorted([(t.lastpt, t) for t in self.tracks if t.alive])]
    tocull = set()
    for (t0,t1) in zip(by_lastpt, by_lastpt[1:]):
      if t0.lastpt == t1.lastpt:
        if 0:
          print "Detected shared point", t0.uniq_track_id, t1.uniq_track_id
          print t0.lastpt, t0.p
          print t1.lastpt, t1.p
          print
        tocull.add(t0)
        tocull.add(t1)
        if len(t0.p) > len(t1.p):
          tocull.add(t0)
        else:
          tocull.add(t1)
    #print "Killing tracks because of shared point:", [t.id for t in tocull]
    self.tracks -= tocull

    for t in self.tracks:
      for (x,y,d) in t.p:
        assert d != 0

    self.timer['tracks'].stop()

  def sba_add_frame(self, frame):
    self.posechain.append((frame,VOLO.frame_pose(frame.id, frame.pose.tolist())))

  def sba_handle_frame(self, frame):
    self.timer['sba'].start()

    # ids is the list of frameids for which we have observations
    ids = set()
    for t in self.tracks:
      #print len(t.p), t.p
      ids |= set(t.id)

    self.sba_add_frame(frame)
    # Only allow frames in the posechain if there exists an observation for that frame
    self.posechain = [ (f,fp) for (f,fp) in self.posechain if f.id in ids ]

    (fix_sz, free_sz, niter) = self.sba
    if len(self.posechain) <= (1 + free_sz):
      fix_sz = 1
      free_sz = len(self.posechain) - 1
    fixed = self.posechain[-(fix_sz + free_sz):-(free_sz)]
    free = self.posechain[-(free_sz):]
    externals = sum([ f.externals for (f,_) in (fixed + free)], [])
    print "SBA:", "fixed", [f.id for (f,_) in fixed], "free", [f.id for (f,_) in free]
    if externals != []:
      fixed += externals
      print "external SBA:", "fixed", [f.id for (f,_) in fixed], "free", [f.id for (f,_) in free]
    self.pe.sba([fp for (_,fp) in fixed], [fp for (_,fp) in free], [ t.sba_track for t in self.tracks ], niter)
    if 0:
      for (f,fp) in free:
        p = Pose()
        p.fromlist(fp.M)
        print f.id, p.xform(0,0,0)
    # copy these corrected poses back into the VO's key and current frames
    to_correct = [ self.keyframe, frame ]
    if self.prev_frame:
      to_correct.append(self.prev_frame)
    #print "Copying poses", [ fp.id for fp in self.posechain ], "to", [ f.id for f in to_correct ]
    for (_,fp) in self.posechain[-(fix_sz + free_sz):]:
      for dst in set(to_correct):
        if fp.id == dst.id:
          dst.pose.fromlist(fp.M)
          print "SBA corrected pose of frame", dst.id, "to", dst.pose.M
    self.timer['sba'].stop()

  def process_frame(self, frame):
    frame.id = self.num_frames

  def add_external_frame(self, frame, fext):
    self.find_keypoints(fext)
    self.find_disparities(fext)
    fext.id = self.ext_frames
    self.ext_frames += 1
    frame.externals.append((fext, VOLO.frame_pose(fext.id, fext.pose.tolist())))

  # just set up the frame with descriptors, no VO processing
  def setup_frame(self, frame):
    self.find_keypoints(frame)
    self.find_disparities(frame)
    frame.id = self.num_frames
    frame.ref_frame_id = None

  # return inliers from a match
  def check_inliers(self, frame1, frame2):
    self.pairs = self.temporal_match(frame1, frame2)
    solution = self.solve(frame1.kp, frame2.kp, self.pairs)
    self.inl = solution[0]

  def change_keyframe(self, newkey, reason):
    print "Change keyframe from", self.keyframe.id, "to", newkey.id, ":", reason
    self.log_keyframes.append(newkey.id)
    self.log_keyposes.append(newkey.pose)
    oldkey = self.keyframe
    self.keyframe = newkey

    if self.sba:
      self.maintain_tracks(oldkey, newkey)
      self.sba_handle_frame(newkey)

  def handle_frame(self, frame):
    """
    Returns pose for *frame*.
    """
    frame.id = self.num_frames
    return self.handle_frame_0(frame)

  def handle_frame_0(self, frame):
    if self.prev_frame:
      # If the key->current is good, use it
      # Otherwise, prev frame becomes the new key

      ref = self.keyframe
      self.pairs = self.temporal_match(ref, frame)
      #if frame.id == 202 and ref.id == 199: self.show_pairs(self.pairs, ref, frame)
      solution = self.solve(ref.features(), frame.features(), self.pairs)
      if solution and solution[0] > 5:
        (inl, rot, shift) = solution
        diff_pose = self.mkpose(rot, shift)
        if self.scavenge:
          solution = self.scavenger(diff_pose, ref, frame)
          if solution and solution[0] > 5:
            (inl, rot, shift) = solution
            diff_pose = self.mkpose(rot, shift)
          else:
            inl = 0
            diff_pose = Pose()
      else:
        inl = 0
        diff_pose = Pose()
      diff_pose.assert_sane()
      self.inl = inl
      self.outl = len(self.pairs) - inl
      frame.diff_pose = diff_pose
#      print "frame", frame.id, "key:", ref.id, "inliers:", inl, "angle_thresh", self.angle_thresh
      newkey_reason = None
      if (self.inl < self.inlier_thresh):
        newkey_reason = "Too few inliers (%d < %d)" % (self.inl, self.inlier_thresh)
      elif Pose().further_than(diff_pose, self.position_thresh, self.angle_thresh):
        newkey_reason = "Passed position thresh"
      if (self.keyframe != self.prev_frame) and newkey_reason:
        self.change_keyframe(self.prev_frame, newkey_reason)
        return self.handle_frame_0(frame)
      Tok = ref.pose
      Tkp = diff_pose
      Top = Tok * Tkp
      frame.pose = Top
      frame.inl = self.inl
      frame.ref_frame_id = ref.id
    else:
      frame.pose = Pose()
      self.keyframe = frame
      self.log_keyframes.append(self.keyframe.id)
      self.log_keyposes.append(self.keyframe.pose)
      frame.inl = 999
      if self.sba:
        self.sba_add_frame(frame)
      frame.ref_frame_id = None
    self.pose.assert_sane()

    self.pose = frame.pose
    self.prev_frame = frame

    self.num_frames  += 1
    self.tot_inliers += self.inl
    self.tot_matches += len(self.pairs)
    self.tot_points  += len(frame.features())

    return self.pose
