import votools as VO
import Image as Image

scratch = " " * (640 * 480)

class ComputedDenseStereoFrame:
  def __init__(self, lf, rf):
    self.rawdata = lf.tostring()
    self.size = lf.size

    # initialize buffer with 'illegal' because dense_stereo leaves pixel untouched it cannot compute - e.g. non-overlapping parts of image
    disp = chr(255) * (2 * self.size[0] * self.size[1])
    VO.dense_stereo(self.rawdata, rf.tostring(), self.size[0], self.size[1], disp)
    self.disp_values = [ord(hi) * 256 + ord(lo) for (lo,hi) in zip(disp[::2], disp[1::2])]

  def lookup_disparity(self, x, y):
    v = self.disp_values[y * self.size[0] + x]
    if v > 0x7fff:
      return None
    else:
      return v / 16.

class DenseStereoFrame:
  """ Dense Stereo directly from the camera.  lf is the left frame, rf is the disparity frame """

  def __init__(self, lf, rf):
    self.rawdata = lf.tostring()
    self.size = lf.size

    # initialize buffer with 'illegal' because dense_stereo leaves pixel untouched it cannot compute - e.g. non-overlapping parts of image
    disp = chr(255) * (2 * self.size[0] * self.size[1])
    VO.dense_stereo(self.rawdata, rf.tostring(), self.size[0], self.size[1], disp)
    self.disp_values = [ord(hi) * 256 + ord(lo) for (lo,hi) in zip(disp[::2], disp[1::2])]

  def lookup_disparity(self, x, y):
    v = self.disp_values[y * self.size[0] + x]
    if v > 0x7fff:
      return None
    else:
      return v / 16.

def do_stereo_sparse(refpat, rgrad, x, y, xim, yim, ftzero, dlen, tfilter_thresh, ufilter_thresh):
  if x < dlen or y < 7 or y > (yim-8):
    return -1
  tiles = [ VO.grab_16x16(rgrad, xim, (x-dlen+1)+i-7, y-7) for i in range(dlen) ]

  ind = VO.sad_search(refpat, tiles, chr(1) * dlen)
  # do interpolation
  if (ind==0 or ind==(dlen-1)):
    return (dlen-ind-1)*16

  c = VO.sad(refpat, tiles[ind])
  p = VO.sad(refpat, tiles[ind+1])
  n = VO.sad(refpat, tiles[ind-1])
  v = float(dlen-ind-1) + (p-n)/(2*(p+n-2*c))
  return (0.5 + 16*v)

class SparseStereoFrame:
  def __init__(self, lf, rf):
    self.lf = lf
    self.rf = rf
    self.rawdata = lf.tostring()
    self.size = lf.size
    (w, h) = self.size

    self.lgrad = " " * (w * h)
    VO.ost_do_prefilter_norm(self.rawdata, self.lgrad, w, h, 31, scratch)

    self.rgrad = " " * (w * h)
    VO.ost_do_prefilter_norm(rf.tostring(), self.rgrad, w, h, 31, scratch)

  def lookup_disparity(self, x, y):
    (w, h) = self.size
    if 1:
      limg = self.lf.tostring()
      rimg = self.rf.tostring()
    else:
      limg,rimg = self.lgrad, self.rgrad
    refpat = VO.grab_16x16(limg, w, x-7, y-7)

    # ftzero         31
    # dlen           64
    # tfilter_thresh 10
    # ufilter_thresh 15

    assert x == int(x)
    assert y == int(y)
    v = VO.ost_do_stereo_sparse(refpat, rimg, x, y, w, h, 31, 64, 10, 15)

    if v < 0:
      return None
    else:
      return v / 16.
