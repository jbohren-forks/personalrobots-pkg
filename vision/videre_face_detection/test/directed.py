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
import numpy

import os

################# CLASS IMGADAPTED ##################
class imgAdapted:
  def __init__(self, i):
    self.i = i
    self.size = i.size
    self.rawdata = i.tostring()

################## CLASS TRACKER CHILD OF CLASS VISUALODOMETER ################
class Tracker(visualodometer.VisualOdometer) :

  def collect_descriptors(self, frame):
    self.timer['descriptor_collection'].start()
    frame.descriptors = [ VO.grab_16x16(frame.rawdata, frame.size[0], p[0]-7, p[1]-7) for p in frame.kp ]
    self.timer['descriptor_collection'].stop()


################## CLASS PEOPLETRACKER #############
class PeopleTracker:
  def __init__(self):
    self.usebag = False
    self.keyframes = None
    self.current_keyframes = None
    self.p = videre_face_detection.people()
    self.cam = None
    self.camparams = None
    self.vo = None
    self.feature_detector = visualodometer.FeatureDetectorStar()
    self.feature_detector.thresh = 3.0
    self.cascade_file = "cascades/haarcascade_frontalface_alt.xml"
    assert os.access(self.cascade_file, os.R_OK)
    self.faces = None
    self.seq = 0
    self.keyframe_maxlength = 1
    self.sad_thresh = 2000.0
    self.num_feats = 40
    self.line_thresh = 10.0
    self.num_scales = 5
    self.feats_to_centers = None
    self.good_points_thresh = 0.5
    self.recent_good_frames = None
    self.recent_good_rects = None
    self.same_key_rgfs = None
    self.pub = None


################### GET_FEATURES ##############################
  def get_features(self, frame, target_points, rect):
    full = Image.fromstring("L",frame.size,frame.rawdata)
    (x,y,w,h) = rect
    incr = 16
    subim = full.crop((x-incr,y-incr,x+w+incr,y+h+incr))
    
    sd = starfeature.star_detector(w+2*incr, h+2*incr, self.num_scales, self.feature_detector.thresh, self.line_thresh)
    results = [ (x1,y1) for (x1,y1,s1,r1) in sd.detect(subim.tostring()) ]
    return [(x-incr+x1,y-incr+y1) for (x1,y1) in results if (incr<x1) and (incr<y1) and (x1<w+incr) and (y1<h+incr)]

################### MAKE_FACE_MODEL ###########################
  def make_face_model(self, window, feat_list) :
    feats_to_center = []
    (wx,wy,ww,wh) = window
    wincenter = (wx+(ww-1)/2,wy+(wh-1)/2)
    for (x,y) in feat_list :
      feats_to_center.append((wincenter[0]-x,wincenter[1]-y))
    return feats_to_center


#################### MEAN SHIFT ###############################
  def mean_shift_sparse(self, start_point, window_size, sparse_pred_list, max_iter, eps) :
    dpoint = [0,0]
    total_weight = 0
    half_width = (window_size[0]-1)/2.0
    half_height = (window_size[1]-1)/2.0
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


################### PARAMETER CALLBACK ########################
  def params(self, pmsg):

    if not self.vo:
      self.cam = camera.VidereCamera(pmsg.data)
      self.vo = Tracker(self.cam)
        


################### IMAGE CALLBACK ############################
  def frame(self, imarray):

    if not self.vo:
      return

    im = imarray.images[1]
    im_py = Image.fromstring("L", (im.width, im.height), im.data)
    im_r = imarray.images[0]
    im_r_py = Image.fromstring("L", (im_r.width, im_r.height), im_r.data)

    # Detect faces on the first frame
    if not self.current_keyframes :
      self.faces = self.p.detectAllFaces(im.data, im.width, im.height, self.cascade_file, 1.0, None, None, True) 
      print self.faces

    sparse_pred_list = []
    ia = SparseStereoFrame(im_py,im_r_py)
    # Track each face
    for iface in range(0,len(self.faces)):
      (x,y,w,h) = self.faces[iface]
      ia.kp2d = self.get_features(ia, self.num_feats, (x, y, w, h))

      self.vo.find_disparities(ia)
      self.vo.collect_descriptors(ia)

      # First frame:
      if not self.current_keyframes :
        self.current_keyframes = [0]
        self.keyframes = [ia]
        self.feats_to_centers = [self.make_face_model( (x,y,w,h), ia.kp2d )]
        self.recent_good_frames = [ia]
        self.recent_good_rects = [[x,y,w,h]]
        self.same_key_rgfs = [True]

      # Later frames
      else :
        # Try matching to the keyframe
        keyframe = self.keyframes[self.current_keyframes[iface]]
        matches = self.vo.temporal_match(keyframe,ia)
   #     print matches
        sadscores = [(VO.sad(keyframe.descriptors[a], ia.descriptors[b])) for (a,b) in matches]
   #     print sadscores

        good_matches = [m for m in sadscores if m < self.sad_thresh]

        # Not enough matches, get a new keyframe
        if len(keyframe.kp)<2 or len(good_matches) < len(keyframe.kp)/2.0 :

          #self.keyframe[self.current_keyframe] = ia
          print "New keyframe"

          # Make a new face model, either from a recent good frame, or from the current image
          if not self.same_key_rgfs[iface] :
            self.feats_to_centers[iface] = self.make_face_model( self.recent_good_rects[iface], self.recent_good_frames[iface].kp2d )
            self.keyframes[self.current_keyframes[iface]] = self.recent_good_frames[iface]
            
            self.same_key_rgfs[iface] = True

          else :
            self.keyframes[self.current_keyframes[iface]] = ia
            self.feats_to_centers[iface] = self.make_face_model( (x,y,w,h), ia.kp2d )
            matches = []
            self.same_key_rgfs[iface] = True

          # Try matching to the NEW keyframe
          keyframe = self.keyframes[self.current_keyframes[iface]]
          matches = self.vo.temporal_match(keyframe,ia)
     #     print matches
          sadscores = [(VO.sad(keyframe.descriptors[a], ia.descriptors[b])) for (a,b) in matches]
     #     print sadscores

          good_matches = [m for m in sadscores if m < self.sad_thresh]


        # Good matches, mark this frame as good
        else:
          self.recent_good_frames[iface] = ia
          self.same_key_rgfs[iface] = False

        # Track
        sparse_pred_list = []
        for imatch in range(0,len(matches)) :
          if sadscores[imatch] < self.sad_thresh :
            (match1,match2) = matches[imatch]
            sparse_pred_list.append( (ia.kp2d[match2][0]+self.feats_to_centers[iface][match1][0], ia.kp2d[match2][1]+self.feats_to_centers[iface][match1][1], 1) )  

    #    print("Sparse pred list", sparse_pred_list)

        new_window = self.mean_shift_sparse( (x+(w-1)/2.0,y+(h-1)/2.0), (w,h), sparse_pred_list, 10, 1.0)
        (nx,ny,nw,nh) = new_window
        # Force the window back into the image.
        dx = max(0,0-nx) + min(0, im.width - nx+nw)
        dy = max(0,0-ny) + min(0, im.height - ny+nh)
        nx += dx
        ny += dy

        self.faces[iface] = [nx, ny, nw, nh]
        self.recent_good_rects[iface] = [nx,ny,nw,nh]

    
        # Output the location of this face center in the 3D camera frame (of the left camera), and rotate 
        # the coordinates to match the robot's idea of the 3D camera frame.
        center_uvd = (nx + (nw-1)/2.0, ny + (nh-1)/2.0, (numpy.average(ia.kp,0))[2] )
        center_camXYZ = self.cam.pix2cam(center_uvd[0], center_uvd[1], center_uvd[2])
        center_robXYZ = (center_camXYZ[2], -center_camXYZ[0], -center_camXYZ[1])
        print ("center_uvd", center_uvd)
        print ("center_camXYZ", center_camXYZ)
        print ("center_robXYZ", center_robXYZ)

        if not self.usebag:
          #PointStamped stamped_point
          stamped_point.point.x = center_robXYZ[0]
          stamped_point.point.y = center_robXYZ[1]
          stamped_point.point.z = center_robYXZ[2]
          stamped_point.header.frame_id = "stereo_block"
          self.pub.publish(PointStamped(stamped_point))
    
      
############ DRAWING ################

    if not self.keyframes :
      bigim_py = im_py
      draw = ImageDraw.Draw(bigim_py)
    else :
      key_im = self.keyframes[self.current_keyframes[0]]
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

    bigim_py.save("tiff/feats%06d.tiff" % self.seq)
  
    self.seq += 1


############# MAIN #############
def main(argv) :
    
  people_tracker = PeopleTracker()

  if len(argv)>0 and argv[0]=="-bag":
    people_tracker.usebag = True

  # Use a bag file
  if people_tracker.usebag :
  
    import rosrecord
    filename = "/wg/stor2/prdata/videre-bags/2008-10-29-15-43-15-topic.bag"
    #filename = "/wg/stor2/prdata/videre-bags/face2.bag"

    num_frames = 0
    for topic, msg in rosrecord.logplayer(filename):
      if num_frames < 500 :
        print topic, msg
        if topic == '/videre/cal_params':
          people_tracker.params(msg)
        elif topic == '/videre/images':
          people_tracker.frame(msg)
          num_frames += 1
        else :
          pass

  # Use new ROS messages, and output the 3D position of the face in the camera frame.
  else :
    rospy.init_node('videre_face_tracker', anonymous=True)
    rospy.TopicSub('/videre/images',ImageArray,people_tracker.frame)
    rospy.TopicSub('/videre/cal_params',String,people_tracker.params)
    people_tracker.pub = rospy.advertise_topic('head_controller/track_point',PointStamped)
    rospy.spin()


if __name__ == '__main__' :
  main(sys.argv[1:])
