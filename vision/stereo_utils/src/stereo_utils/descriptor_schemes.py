"""
:mod:`stereo_utils.descriptor_schemes` --- Collection of descriptor schemes
===========================================================================
"""
import stereo_utils.lowlevel as LO
from stereo_utils.timer import TimedClass
import vop
import os

class DescriptorScheme(TimedClass):
  """
  A descriptor scheme has:

  * a way to generate descriptors (also called signatures) from keypoints.  This is the method :meth:`collect0`.
  * a comparison function that finds correspondences between two lists of descriptors.  This is the method :meth:`match0`.
  """

  timing = []

  def __init__(self):
    TimedClass.__init__(self, self.timing + ['Match'])

  def name(self):
    return self.__class__.__name__

  def desc2matcher(self, descriptors):
    return descriptors

  def collect0(self, frame, kp):
    """
    Given a *frame* :class:`stereo_utils.stereo.Frame` and keypoints *kp*,
    return a list of descriptors.
    """
    pass

  def match0(self, af0kp, af0descriptors, af1kp, af1descriptors):
    """
    Given keypoints and descriptors for af0 and af1, returns
    a list of pairs *(a,b)*, where
    *a* is an index into this *af0*'s keypoints, and
    *b* is an index into *af1*'s keypoints.
    """

    if af0kp == [] or af1kp == []:
      return []
    self.calls += 1
    self.timer['Match'].start()
    Xs = vop.array([k[0] for k in af1kp])
    Ys = vop.array([k[1] for k in af1kp])
    pairs = []
    matcher = self.desc2matcher(af1descriptors)
    for (i,(ki,di)) in enumerate(zip(af0kp, af0descriptors)):
      predX = (abs(Xs - ki[0]) < 64)
      predY = (abs(Ys - ki[1]) < 32)
      hits = vop.where(predX & predY, 1, 0).tostring()
      best = self.search(di, matcher, hits)
      if best != None:
        pairs.append((i, best[0], best[1]))
    self.timer['Match'].stop()
    return pairs

scratch = " " * (640 * 480)

class DescriptorSchemeSAD(DescriptorScheme):
  """
  SAD is sum of absolute differences. See
  http://en.wikipedia.org/wiki/Sum_of_absolute_differences.  This
  implementation uses a window size of 16x16 pixels.
  """

  def collect0(self, frame, kp):
    lgrad = " " * (frame.size[0] * frame.size[1])
    LO.ost_do_prefilter_norm(frame.rawdata, frame.lgrad, frame.size[0], frame.size[1], 31, scratch)
    return [ LO.grab_16x16(frame.lgrad, frame.size[0], p[0]-7, p[1]-7) for p in kp ]

  def search(self, di, descriptors, hits):
      i = LO.sad_search(di, descriptors, hits)
      if i == None:
        return None
      else:
        return (i, 0)

import calonder

class DescriptorSchemeCalonder(DescriptorScheme):
  """
  Calonder is a random tree classifier.
  """
  timing = [ 'BuildMatcher', 'Collect' ]

  def __init__(self):
    self.cl = calonder.classifier()
    #self.cl.setThreshold(0.0)
    filename = '/u/prdata/calonder_trees/current.rtc'
    assert os.access(filename, os.R_OK)
    self.cl.read(filename)
    DescriptorScheme.__init__(self)

  def collect0(self, frame, kp):
    self.timer['Collect'].start()
    r = self.cl.getSignatures(frame.size, frame.rawdata, [ (x,y) for (x,y,d) in kp ])
    self.timer['Collect'].stop()
    return r

  def desc2matcher(self, descriptors):
    self.timer['BuildMatcher'].start()
    matcher = calonder.BruteForceMatcher(self.cl.dimension())
    for sig in descriptors:
      matcher.addSignature(sig)
    self.timer['BuildMatcher'].stop()
    return matcher

  def search(self, di, matcher, hits):
    r = matcher.findMatch(di, hits)
    return r
