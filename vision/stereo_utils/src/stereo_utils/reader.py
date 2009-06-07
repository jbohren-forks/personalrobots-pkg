"""
:mod:`stereo_utils.reader` --- Generic image sequence reader
============================================================
"""

import os

import Image
import yaml
import cv

import rosrecord
import rospy
from stereo_utils import camera

class dcamImage:
  def __init__(self, m):
    if hasattr(m, "byte_data"):
      ma = m.byte_data
      self.data = ma.data
    else:
      ma = m.uint8_data # MultiArray
      self.data = ma.data
    d = ma.layout.dim
    assert d[0].label == "height"
    assert d[1].label == "width"
    self.size = (d[1].size, d[0].size)
    self.mode = "L"

  def tostring(self):
    return self.data

class reader:
  """
  Is an iterator that yields a sequence of tuples *(Cam, lf, rf)* for every frame in *sourcename*, where

  *Cam*
      An instance of a :class:`stereo_utils.camera.Camera`
  *lf*
      Left image
  *rf*
      Right image

  *lf* and *rf* are suitable for use with :mod:`stereo_utils.stereo`.

  *sourcename* may be a
  `ROS <http://pr.willowgarage.com/wiki/ROS>`_ bag, or a directory containing image files.  For example::

     for (Cam,L,R) in stereo_utils.reader.reader("foo.bag"):
       stereo_frame = SparseStereoFrame(L, R)
  """

  def __init__(self, sourcename):
    if os.path.isdir(sourcename):
      self.next = self.next_from_dir
      self.dc = yaml.load(open("%s/sequence_parameters" % sourcename).read())
      self.cam = camera.DictCamera(self.dc)
      self.f = int(self.dc['FirstFrame'])
    else:
      self.gen = rosrecord.logplayer(sourcename)
      self.next = self.next_from_bag
    self.sourcename = sourcename

  def seek(self, frame):
    self.f = int(self.dc['FirstFrame']) + frame

  def from_msg(self, m):
    return dcamImage(m)

  def from_file(self, filename):
    return Image.open(filename)

  def next_from_bag(self):

    while True:
      topic, msg, t = self.gen.next()
      if rospy.is_shutdown():
        break
      if topic.endswith("stereo/raw_stereo"):
        cam = camera.StereoCamera(msg.right_info)
        return cam, self.from_msg(msg.left_image), self.from_msg(msg.right_image)
    raise StopIteration

  def __iter__(self):
    return self

  def next_from_dir(self):

    Lname = self.sourcename + "/" + self.dc['LeftFilename'] % self.f
    Rname = self.sourcename + "/" + self.dc['RightFilename'] % self.f

    if not os.access(Lname, os.R_OK) or not os.access(Rname, os.R_OK):
      if 'LastFrame' in self.dc:
        self.f += 1
        return self.next()
      else:
        raise StopIteration
    L = self.from_file(Lname)
    R = self.from_file(Rname)

    if ('LastFrame' in self.dc) and (self.f == int(self.dc['LastFrame'])):
      raise StopIteration
    self.f += 1
    return self.cam, L, R


class CVreader(reader):

  def from_msg(self, m):
    ma = m.uint8_data # MultiArray
    if len(ma.layout.dim) == 0:
      return None
    dim = dict([ (d.label,d.size) for d in ma.layout.dim ])
    (w,h) = (dim['width'], dim['height'])
    im = cv.CreateImageHeader((w,h), cv.IPL_DEPTH_8U, dim['channel'])
    cv.SetData(im, ma.data, dim['width'] * dim['channel'])
    return im

  def from_file(self, filename):
    return cv.LoadImage(filename, cv.CV_LOAD_IMAGE_UNCHANGED)
