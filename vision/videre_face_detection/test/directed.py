import rostools
rostools.update_path('videre_face_detection')
import rostest
import videre_face_detection
import visualodometer
import camera
import std_msgs.msg
import visual_odometry as VO

import sys
sys.path.append('lib')

import Image
import ImageChops
import ImageDraw

import random
import time
import math

import os

class imgAdapted:
  def __init__(self, i):
    self.i = i
    self.size = i.size
    self.rawdata = i.tostring()

class PeopleTracker:
  def __init__(self):
    self.keyframe = []
    self.current_keyframe = -1
    self.p = videre_face_detection.people()
    self.cam = camera.Camera((389.0, 389.0, 89.23, 323.42, 323.42, 274.95))
    self.vo = visualodometer.VisualOdometer(self.cam)
    self.feature_detector = visualodometer.FeatureDetectorStar()
    self.feature_detector.thresh = 5
    self.cascade_file = "cascades/haarcascade_frontalface_alt.xml"
    assert os.access(self.cascade_file, os.R_OK)
    self.faces = None
    self.seq = 0
    self.keyframe_maxlength = 1
    self.sad_thresh = 500

  def frame(self, imarray):
    im = imarray.images[1]
    im_py = Image.fromstring("L", (im.width, im.height), im.data)

    if self.current_keyframe == -1 :
      self.faces = self.p.detectAllFaces(im.data, im.width, im.height, self.cascade_file, 1.0, None, None, True) 
      print self.faces
      self.faces = [ (x-16,y-16,w+32,h+32) for ( x,y,w,h ) in self.faces ]

    for (x, y, w, h) in self.faces:
      subim = im_py.crop((x, y, x+w, y+h))
      ia = imgAdapted(subim)
      ia.kp2d = self.feature_detector.get_features(ia, 20)
      ia.kp2d = [(x1,y1) for (x1,y1) in ia.kp2d if (16<x1) and (16<y1) and (x1<(w-32)) and (y1<(h-32))] 
      #print ia.kp2d
      ia.kp = ia.kp2d
      self.vo.collect_descriptors(ia)
      if self.current_keyframe > -1 :
        matches = self.vo.temporal_match(self.keyframe[self.current_keyframe],ia)
        print matches
        print [VO.sad(self.keyframe[self.current_keyframe].descriptors[a], ia.descriptors[b]) for (a,b) in matches] 
        print len(matches)
        #if len(matches) < len(self.keyframe[self.current_keyframe].kp)/2 :
        #  self.keyframe[self.current_keyframe] = ia
        #  print "New keyframe"
      else :
        self.current_keyframe = 0
        self.keyframe.append(ia)
      
    draw = ImageDraw.Draw(im_py)
    for (x,y,w,h) in self.faces:
      draw.line((x,y,x+w,y), fill=255)
      draw.line((x,y+h,x+w,y+h), fill=255)
      draw.line((x,y,x,y+h), fill=255)
      draw.line((x+w,y,x+w,y+h), fill=255)
      
      for (x1, y1) in ia.kp :
        draw.line((x+x1-1,y+y1-1,x+x1+1,y+y1+1), fill=255)
        draw.line((x+x1-1,y+y1+1,x+x1+1,y+y1-1), fill=255)

    im_py.save("feats%06d.tiff" % self.seq)
  
    self.seq += 1


people_tracker = PeopleTracker()

import rosrecord

filename = "/wg/stor2/prdata/videre-bags/face1.bag"

for topic, msg in rosrecord.logplayer(filename):
  print topic, msg
  if topic == '/videre/images':
    people_tracker.frame(msg)

