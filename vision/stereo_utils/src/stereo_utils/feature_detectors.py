"""
:mod:`stereo_utils.feature_detector` --- Collection of feature detectors
========================================================================
"""
import stereo_utils.lowlevel as LO
from stereo_utils.timer import TimedClass
import fast
import cv

class FeatureDetector(TimedClass):
  """ Base class for all feature detectors.  *target_count* is desired number of features. """
  timing = []

  def name(self):
    return self.__class__.__name__

  def __init__(self, target_count):
    self.thresh = self.default_thresh
    self.cold = True
    self.target_count = target_count

    TimedClass.__init__(self, self.timing + ['detect'])

  def detect(self, frame):
    """
    Return a list of *(x,y)* coordinates.  The list is at most
    *target_count* in length.
    """
    self.timer['detect'].start()
    self.calls += 1
    features = self.get_features(frame, self.target_count)
    if len(features) < (self.target_count * 0.5) or len(features) > (self.target_count * 2.0):
        (lo,hi) = self.threshrange
        for i in range(7):
          self.thresh = 0.5 * (lo + hi)
          features = self.get_features(frame, self.target_count)
          if len(features) < self.target_count:
            hi = self.thresh
          if len(features) > self.target_count:
            lo = self.thresh
        self.thresh = 0.5 * (lo + hi)

    # Try to be a bit adaptive for next time
    if len(features) > (self.target_count * 1.1):
        self.thresh *= 1.05
    if len(features) < (self.target_count * 0.9):
        self.thresh *= 0.95
    self.timer['detect'].stop()
    return features

# Feature detectors that return features in order (i.e. strongest first)
# can be simpler.  Just always keep the threshold high enough to give
# too many responses, and take the top N.

class FeatureDetectorOrdered(FeatureDetector):
  """ Base class for feature detectors that return their features ranked by response """
  timing = ['get_features/False', 'get_features/True' ]

  def detect(self, frame):

    self.calls += 1
    self.timer['detect'].start()
    self.timer['get_features/False'].start()
    features = self.get_features(frame, self.target_count)
    self.timer['get_features/False'].stop()
    # Too few features, so lower threshold
    while (len(features) < self.target_count) and (self.thresh > self.threshrange[0]):
      self.thresh = float(max(self.threshrange[0], self.thresh / 2))
      self.timer['get_features/False'].start()
      features = self.get_features(frame, self.target_count)
      self.timer['get_features/False'].stop()
    # If starving, rerun 
    if (len(features) < 100) and (self.thresh <= self.threshrange[0]):
      self.timer['get_features/True'].start()
      features = self.get_features(frame, self.target_count, True)
      self.timer['get_features/True'].stop()

    # Try to be a bit more adaptive for next time
    if len(features) > (self.target_count * 2):
      self.thresh *= 2
    if len(features) < (self.target_count * 1.25):
      self.thresh *= 0.95
    self.timer['detect'].stop()
    return features[:self.target_count]

def FAST(imdata, xsize, ysize, thresh, barrier = 9):
  kp = fast.fast(imdata, xsize, ysize, barrier, int(thresh))
  return sorted(fast.nonmax(kp), key = lambda x:(x[2],x[0],x[1]), reverse = True)

class FeatureDetectorFast(FeatureDetectorOrdered):
  """
  Feature detector using Edward Rosten's FAST corner detection
  http://svr-www.eng.cam.ac.uk/~er258/work/fast.html
  """
  default_thresh = 2000
  threshrange = (0.5,30000)

  def get_features(self, frame, target_points, starving = False):
    assert len(frame.rawdata) == (frame.size[0] * frame.size[1])
    if starving:
      barrier = 3
    else:
      barrier = 9
    feat = FAST(frame.rawdata, frame.size[0], frame.size[1], self.thresh, barrier)
    return [ (x,y) for (x,y,r) in feat if (16 <= x and x <= (frame.size[0]-16) and (16 <= y) and y < (frame.size[1]-16)) ]

class FeatureDetector4x4:

  def __init__(self, fd):
    self.fds = [ fd() for i in range(16) ]

  def name(self):
    return "4x4 " + self.fds[0].__class__.__name__

  def detect(self, frame, target_points):
    master = Image.fromstring("L", frame.size, frame.rawdata)
    allpts = []
    xbase = 16
    ybase = 16
    w = frame.size[0] - 32
    h = frame.size[1] - 32
    for x in range(4):
      for y in range(4):
        xleft = xbase + x * (w/4)
        ytop = ybase + y * (h/4)
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
  """
  Feature detector using Harris Corners via OpenCV's `GoodFeaturesToTrack` function.
  """

  default_thresh = 0.130
  threshrange = (1e-4,1.0)

  def get_features(self, frame, target_points):
    arr = cv.CreateImage(frame.size, cv.IPL_DEPTH_8U, 1)
    cv.SetData(arr, frame.rawdata, frame.size[0])
    eig_image = cv.CreateImage(frame.size, cv.IPL_DEPTH_32F, 1)
    temp_image = cv.CreateImage(frame.size, cv.IPL_DEPTH_32F, 1)
    pts = cv.GoodFeaturesToTrack(arr, eig_image, temp_image, target_points, self.thresh, 2, use_harris = 1)
    return [(int(x), int(y)) for (x,y) in pts]

import starfeature

class FeatureDetectorStar(FeatureDetectorOrdered):
  """ Feature detector using the Star operator """
  default_thresh = 30.0
  threshrange = (1,64)
  line_thresh = 10.0

  def get_features(self, frame, target_points):
    sd = starfeature.star_detector(frame.size[0], frame.size[1], 5, self.thresh, self.line_thresh)
    by_response = sorted(sd.detect(frame.rawdata), key = lambda x:(abs(x[3]),x[0],x[1]), reverse = True)
    return [ (x,y) for (x,y,s,r) in by_response ]
