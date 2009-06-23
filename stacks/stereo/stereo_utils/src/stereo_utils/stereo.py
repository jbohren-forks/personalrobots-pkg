"""
:mod:`stereo_utils.stereo` --- Various types of stereo frames
=============================================================
"""

import stereo_utils.lowlevel as LO
import Image as Image

scratch,_ = LO.buffer(640 * 480)

class Frame:
  """ 
  base class for all stereo frames.  Mandatory kwargs *feature_detector* and *descriptor_scheme* specify classes for feature detection and descriptor matching respectively.
  Optional kwarg *disparity_range* specifies the disparity range, default is 64.
  """

  def __init__(self, **kwargs):
    self.feat = None
    self.feature_detector = kwargs['feature_detector']
    self.desc = None
    self.descriptor_scheme = kwargs['descriptor_scheme']
    self.disparity_range = kwargs.get('disparity_range', 64)

  def features(self):
    """
    Calls the frame's feature detector to give a list of keypoints,
    uses method lookup_disparity to obtain a list of disparities and
    returns a list of (x,y,d) stereo keypoints.

    This method caches its return value so that subsequent calls do no computation.
    """

    if not self.feat:
      kp2d = self.feature_detector.detect(self)
      disparities = [self.lookup_disparity(x,y) for (x,y) in kp2d]
      self.feat = [ (x,y,z) for ((x,y),z) in zip(kp2d, disparities) if z]
    return self.feat

  def descriptors(self):
    """
    Calls the frame's descriptor_scheme method to collect a list of signatures for
    the frame's keypoints.

    This method caches its return value so that subsequent calls do no computation.
    """

    if not self.desc:
      self.desc = self.descriptor_scheme.collect0(self, self.features())
    return self.desc

  def match(self, other):
    """
    Returns a list of best matches from this frame's keypoints to the other frames.  Return is a list of pairs (a,b), where a is an index into this frame's keypoints, and b is an index into *other*'s keypoints.
    """

    assert self.descriptor_scheme == other.descriptor_scheme
    return self.descriptor_scheme.match0(self.features(), self.descriptors(), other.features(), other.descriptors())

class ComputedDenseStereoFrame(Frame):
  """ Dense stereo computed from left and right images.  *lf* is the
  left frame, *rf* is the right frame.
  """
  def __init__(self, lf, rf):
    self.rawdata = lf.tostring()
    self.size = lf.size

    # initialize buffer with 'illegal' because dense_stereo leaves pixel
    # untouched it cannot compute - e.g. non-overlapping parts of image
    disp = chr(255) * (2 * self.size[0] * self.size[1])
    LO.dense_stereo(self.rawdata, rf.tostring(), self.size[0], self.size[1], disp)
    self.disp_values = [ord(hi) * 256 + ord(lo) for (lo,hi) in zip(disp[::2], disp[1::2])]

  def lookup_disparity(self, x, y):
    v = self.disp_values[y * self.size[0] + x]
    if v > 0x7fff:
      return None
    else:
      return v / 16.

class DenseStereoFrame(Frame):
  """ Dense stereo directly from the camera.  *lf* is the left frame, *rf*
  is the disparity frame.  Computes disparity over the entire image.
  """

  def __init__(self, lf, rf, **kwargs):
    Frame.__init__(self, **kwargs)
    self.rawdata = lf.tostring()
    self.size = lf.size

    (w, h) = self.size
    self.lgrad = " " * (w * h)
    LO.ost_do_prefilter_norm(self.rawdata, self.lgrad, w, h, 31, scratch)

    # initialize buffer with 'illegal' because dense_stereo leaves pixel untouched it cannot compute - e.g. non-overlapping parts of image
    disp = chr(255) * (2 * self.size[0] * self.size[1])
    LO.dense_stereo(self.rawdata, rf.tostring(), self.size[0], self.size[1], disp)
    self.disp_values = [ord(hi) * 256 + ord(lo) for (lo,hi) in zip(disp[::2], disp[1::2])]

  def lookup_disparity(self, x, y):
    v = self.disp_values[y * self.size[0] + x]
    if v > 0x7fff:
      return None
    else:
      return v / 16.

class SparseStereoFrame(Frame):
  """ *lf* is the left frame, *rf* is the right frame.  Only computes disparity for pixels as needed. """

  def __init__(self, lf, rf, use_grad_img = True, **kwargs):
    Frame.__init__(self, **kwargs)
    self.externals = []
    self.lf = lf
    self.rf = rf
    self.rawdata = lf.tostring()
    self.size = lf.size
    self.use_grad_img = use_grad_img
    (w, h) = self.size

    self.buffers = []

    self.lgrad,b = LO.buffer(w * h)
    self.buffers.append(b)
    LO.ost_do_prefilter_norm(self.rawdata, self.lgrad, w, h, 31, scratch)

    self.rgrad,b = LO.buffer(w * h)
    self.buffers.append(b)
    LO.ost_do_prefilter_norm(rf.tostring(), self.rgrad, w, h, 31, scratch)

  def lookup_disparity(self, x, y):
    (w, h) = self.size
    if self.use_grad_img:
      limg,rimg = self.lgrad, self.rgrad
    else:
      limg = self.lf.tostring()
      rimg = self.rf.tostring()
    refpat = LO.grab_16x16(limg, w, x-7, y-7)

    # ftzero         31
    # dlen           64
    # tfilter_thresh 10
    # ufilter_thresh 15

    assert x == int(x)
    assert y == int(y)
    #dlen = max(16, min(x, 64))
    dlen = self.disparity_range
    v = LO.ost_do_stereo_sparse(refpat, rimg, x, y, w, h, 31, dlen, 10, 15)

    if v < 0:
      return None
    else:
      return v / 16.

  def __del__(self):
    for b in self.buffers:
      LO.release(b)
