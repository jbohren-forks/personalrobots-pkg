import rostools
rostools.update_path('videre_face_detection')
import rostest
import videre_face_detection
import visualodometer
import camera
import std_msgs.msg
import visual_odometry as VO
import starfeature

from stereo import SparseStereoFrame

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

class Tracker(visualodometer.VisualOdometer) :

  def collect_descriptors(self, frame):
    self.timer['descriptor_collection'].start()
    frame.descriptors = [ VO.grab_16x16(frame.rawdata, frame.size[0], p[0]-7, p[1]-7) for p in frame.kp ]
    self.timer['descriptor_collection'].stop()



class PeopleTracker:
  def __init__(self):
    self.keyframe = []
    self.current_keyframe = -1
    self.p = videre_face_detection.people()
    self.cam = camera.Camera((389.0, 389.0, 89.23, 323.42, 323.42, 274.95))
    self.vo = Tracker(self.cam)
    self.feature_detector = visualodometer.FeatureDetectorStar()
    self.feature_detector.thresh = 3
    self.cascade_file = "cascades/haarcascade_frontalface_alt.xml"
    assert os.access(self.cascade_file, os.R_OK)
    self.faces = []
    self.seq = 0
    self.keyframe_maxlength = 1
    self.sad_thresh = 2000
    self.num_feats = 40
    self.line_thresh = 10.0
    self.num_scales = 5
    self.feats_to_center = []
    self.good_points_thresh = 1/2

  def get_features(self, frame, target_points, rect):
    full = Image.fromstring("L",frame.size,frame.rawdata)
    (x,y,w,h) = rect
    incr = 16
    subim = full.crop((x-incr,y-incr,x+w+incr,y+h+incr))
    
    sd = starfeature.star_detector(w+2*incr, h+2*incr, self.num_scales, self.feature_detector.thresh, self.line_thresh)
    results = [ (x1,y1) for (x1,y1,s1,r1) in sd.detect(subim.tostring()) ]
    return [(x-incr+x1,y-incr+y1) for (x1,y1) in results if (incr<x1) and (incr<y1) and (x1<w+incr) and (y1<h+incr)]

  def make_face_model(self, window, feat_list) :
    feats_to_center = []
    (wx,wy,ww,wh) = window
    wincenter = (wx+(ww-1)/2,wy+(wh-1)/2)
    for (x,y) in feat_list :
      feats_to_center.append((wincenter[0]-x,wincenter[1]-y))
    return feats_to_center

  def mean_shift_sparse(self, start_point, window_size, sparse_pred_list, max_iter, eps) :
    dpoint = [0,0]
    total_weight = 0
    half_width = (window_size[0]-1)/2
    half_height = (window_size[1]-1)/2
    new_point = start_point
    # For each iteration
    for iter in range(0,max_iter) :
      # Add each prediction in the kernel to the displacement
      for (x,y,weight) in sparse_pred_list :
        diffxy = (x-new_point[0], y-new_point[1])
        if (abs(diffxy[0])<=half_width) and (abs(diffxy[1])<=half_height) :
          total_weight += weight
          dpoint[0] += weight*diffxy[0]
          dpoint[1] += weight*diffxy[1]
      # If there weren't any predictions in the kernel, return the old point.
      if total_weight == 0 :
        return (new_point[0]-half_width,new_point[1]-half_height,window_size[0],window_size[1])
      # Otherwise, move the point
      else :
        dpoint[0] /= total_weight
        dpoint[1] /= total_weight
        new_point = [new_point[0] + dpoint[0], new_point[1] + dpoint[1]]
        
      # If the displacement was small, return the point
      if dpoint[0]*dpoint[0]+dpoint[1]*dpoint[1] <= eps*eps :
        return (new_point[0]-half_width,new_point[1]-half_height,window_size[0],window_size[1])

    # Reached the maximum number of iterations, return
    return (new_point[0]-half_width,new_point[1]-half_height,window_size[0],window_size[1])


  def frame(self, imarray):

    if self.seq>100 :
      self.seq += 1
      return

    im = imarray.images[1]
    im_py = Image.fromstring("L", (im.width, im.height), im.data)
    im_r = imarray.images[0]
    im_r_py = Image.fromstring("L", (im_r.width, im_r.height), im_r.data)

    if self.current_keyframe == -1 :
      self.faces = self.p.detectAllFaces(im.data, im.width, im.height, self.cascade_file, 1.0, None, None, True) 
      print self.faces
      #self.faces = [ (x-16,y-16,w+32,h+32) for ( x,y,w,h ) in self.faces ]

    sparse_pred_list = []
    for iface in range(0,len(self.faces)):
      (x,y,w,h) = self.faces[iface]
      #subim = im_py.crop((x, y, x+w, y+h))
      #ia = imgAdapted(subim)
      ia = SparseStereoFrame(im_py,im_r_py)  #imgAdapted(im_py)
      ia.kp2d = self.get_features(ia, self.num_feats, (x, y, w, h))

      
      #ia.kp2d = [(x1,y1) for (x1,y1) in ia.kp2d if (16<x1) and (16<y1) and (x1<(w-32)) and (y1<(h-32))] 
      #ia.kp = ia.kp2d
      self.vo.find_disparities(ia)
      print ia.kp
      self.vo.collect_descriptors(ia)
      if self.current_keyframe > -1 :
        matches = self.vo.temporal_match(self.keyframe[self.current_keyframe],ia)
        print matches
        sadscores = [(VO.sad(self.keyframe[self.current_keyframe].descriptors[a], ia.descriptors[b])) for (a,b) in matches]
        print sadscores
        print len(matches)

        good_matches = [m for m in sadscores if m < self.sad_thresh]

        if len(self.keyframe[self.current_keyframe].kp)<2 or len(good_matches) < len(self.keyframe[self.current_keyframe].kp)/2 :
          self.keyframe[self.current_keyframe] = ia
          print "New keyframe"
          matches = []
          self.feats_to_center = self.make_face_model( (x,y,w,h), ia.kp2d )
        else:
          sparse_pred_list = []
          for imatch in range(0,len(matches)) :
            if sadscores[imatch] < self.sad_thresh :
              (match1,match2) = matches[imatch]
              sparse_pred_list.append( (ia.kp2d[match2][0]+self.feats_to_center[match1][0], ia.kp2d[match2][1]+self.feats_to_center[match1][1], 1) )  

          print("Sparse pred list")
          print sparse_pred_list

          new_window = self.mean_shift_sparse( (x+(w-1)/2,y+(h-1)/2), (w,h), sparse_pred_list, 10, 1)
          print("new window")
          print new_window
          print ("im size")
          print (im.width, im.height)
          (nx,ny,nw,nh) = new_window
#          # Force the window back into the image.
          dx = max(0,0-nx) + min(0, im.width - nx+nw)
          dy = max(0,0-ny) + min(0, im.height - ny+nh)
          nx += dx
          ny += dy
          print "New window adj"
          print (nx,ny,nw,nh)

          self.faces[iface] = [nx, ny, nw, nh]
      else :
        self.current_keyframe = 0
        self.keyframe.append(ia)
        self.feats_to_center = self.make_face_model( (x,y,w,h), ia.kp2d )
      
    key_im = self.keyframe[self.current_keyframe]
    keyim_py = Image.fromstring("L", key_im.size, key_im.rawdata)
    bigim_py = Image.new("RGB",(im_py.size[0]+key_im.size[0], im_py.size[1]))
    bigim_py.paste(keyim_py.convert("RGB"),(0,0))
    bigim_py.paste(im_py,(key_im.size[0]+1,0))
    draw = ImageDraw.Draw(bigim_py)

    for (x,y,w,h) in self.faces:
      draw.rectangle((x,y,x+w,y+h),outline=(255,0,0))
      draw.rectangle((x+key_im.size[0],y,x+w+key_im.size[0],y+h),outline=(255,0,0))

    for (x,y) in ia.kp2d:
      draw.line((x-1+key_im.size[0], y-1, x+1+key_im.size[0], y+1), fill=(255,0,0))
      draw.line((x+1+key_im.size[0], y-1, x-1+key_im.size[0], y+1), fill=(255,0,0))

    for (x,y) in key_im.kp2d :
      draw.line((x-1, y-1, x+1, y+1), fill=(255,0,0))
      draw.line((x+1, y-1, x-1, y+1), fill=(255,0,0))

    if self.seq > 0 :

      for (x,y,w) in sparse_pred_list :
        draw.line((x-1, y-1, x+1, y+1), fill=(0,0,255))
        draw.line((x+1, y-1, x-1, y+1), fill=(0,0,255))
        draw.line((x-1+key_im.size[0], y-1, x+1+key_im.size[0], y+1), fill=(0,0,255))
        draw.line((x+1+key_im.size[0], y-1, x-1+key_im.size[0], y+1), fill=(0,0,255))


      for ((m1,m2), score) in zip(matches,sadscores) :
        if score > self.sad_thresh :
          color = (255,0,0)
        else :
          color = (0,255,0)
        draw.line((key_im.kp2d[m1][0], key_im.kp2d[m1][1], ia.kp2d[m2][0]+key_im.size[0], ia.kp2d[m2][1]), fill=color)

    bigim_py.save("feats%06d.tiff" % self.seq)
  
    self.seq += 1


people_tracker = PeopleTracker()

import rosrecord

filename = "/wg/stor2/prdata/videre-bags/face2.bag"

for topic, msg in rosrecord.logplayer(filename):
  print topic, msg
  if topic == '/videre/images':
    people_tracker.frame(msg)

